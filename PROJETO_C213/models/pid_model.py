import numpy as np
import scipy.io as sio
from control import tf, feedback, step_response, pade

class PIDModel:
    """
    Model (Camada M - Modelo).
    
    Encapsula toda a lógica de cálculo do projeto: carregamento de dados, 
    identificação FOPDT, regras de sintonia PID e simulação em malha fechada.
    """
    
    def __init__(self):
        """Inicializa os parâmetros do modelo FOPDT e variáveis de estado."""
        # Parâmetros FOPDT, inicializados com np.nan para manipulação segura por funções NumPy
        self.k, self.tau, self.theta = np.nan, np.nan, np.nan
        
        # Armazena os dados brutos e parâmetros derivados da curva de reação
        self.t, self.u, self.y = None, None, None
        self.y0, self.du = np.nan, np.nan  # y0: nível inicial; du: magnitude do degrau
        self.den_norm = np.nan            # dy: variação total da saída (y1 - y0)
        self.method_id = "N/A"            # Método de identificação vencedor (Smith ou Sundaresan)
        
        # Parâmetros do controlador PID (último valor calculado ou manual)
        self.Kp, self.Ti, self.Td = 0.0, 0.0, 0.0
        
        # Constantes das correlações ITAE para sintonia (minimiza erro em trânsito)
        self.ITAE_CONST = [0.965, -0.85, 0.796, -0.147, 0.308, 0.929]
        
    # --- Funções Auxiliares de Identificação e Simulação ---
    
    def _time_at_norm(self, percent):
        """Calcula o tempo (t) em que a resposta normalizada atinge uma dada porcentagem (%)."""
        if self.t is None or not np.isfinite(self.den_norm) or abs(self.den_norm) < 1e-12:
            return np.nan
        
        # Normalização da saída para faixa [0, 1]
        y_norm = (self.y - self.y0) / self.den_norm
        
        # Encontra o primeiro índice de tempo que atinge ou excede o percentual
        id = np.where(y_norm >= percent)[0]

        if len(id) > 0:
            return self.t[id[0]]
        else:
            return np.nan

    def _simulate_fopdt(self, k, tau, theta, t_data, y0, du):
        """Simula a resposta da planta FOPDT G(s) = k/(tau*s + 1) * e^(-theta*s)."""
        
        # Validação da estabilidade e causalidade do modelo
        if not (np.isfinite(k) and np.isfinite(tau) and np.isfinite(theta)) or tau <= 0 or theta < 0:
            return np.full_like(t_data, np.nan, dtype=float)
            
        # Função de Transferência de Primeira Ordem
        G = tf([k], [tau, 1])

        # Aproximação de Padé de 1ª ordem para o termo de atraso de transporte
        nd, dd = pade(theta, 1) 
        Gd = tf(nd, dd)
        
        # Planta completa Gp(s)
        Gp = G * Gd
        
        # Simulação da resposta ao degrau unitário
        t_sim, y_step = step_response(Gp, T=t_data)
        
        # Escala e desloca a resposta para o degrau real (du) e nível inicial (y0)
        y_hat = y0 + du * y_step

        return y_hat

    def _rmse(self, y_true, y_hat):
        """Calcula o Erro Quadrático Médio da Raiz (RMSE), usado como EQM."""
        if np.isnan(y_hat).any() or len(y_true) != len(y_hat):
            return np.inf

        erro = y_true - y_hat
        # Fórmula: sqrt(sum(erro^2) / n)
        return np.sqrt(np.mean(erro**2))

    def _get_pid_tf(self):
        """Monta a Função de Transferência do Controlador PID na forma paralela."""
        Kp, Ti, Td = self.Kp, self.Ti, self.Td
        
        if (not np.isfinite(Kp)) or (not np.isfinite(Ti)) or (not np.isfinite(Td)):
            return tf([0], [1])
            
        # Condição para evitar singularidade e controlador nulo
        if Ti <= 0 or (Kp == 0.0 and Ti == 0.0 and Td == 0.0):
             return tf([0], [1]) 

        # Gc(s) = Kp + Ki/s + Kd*s  =>  (Kp*Td*s^2 + Kp*s + Kp/Ti) / s
        num = [Kp * Td, Kp, Kp / Ti]
        den = [1, 0]
        return tf(num, den)

    # --- Métodos Públicos (Ações Principais) ---

    def load_data(self, filepath):
        """Carrega dados da curva de reação do arquivo .mat e calcula o ganho estático (k)."""
        try:
            data = sio.loadmat(filepath)
            self.t = np.ravel(data['tiempo'])
            self.u = np.ravel(data['entrada'])
            self.y = np.ravel(data['salida'])
            
            # Determinação dos níveis de entrada e saída
            u0, u1 = np.mean(self.u[:50]), np.mean(self.u[-50:])
            self.y0, y1 = np.mean(self.y[:50]), np.mean(self.y[-50:])
            
            du_raw = (u1 - u0)
            self.du = du_raw if abs(du_raw) > 1e-12 else float(np.mean(self.u))
            
            dy = (y1 - self.y0)
            self.k = dy / self.du if abs(self.du) > 1e-12 else np.nan
            self.den_norm = dy
            
            if not np.isfinite(self.k):
                self.k = np.nan

            return True
        except Exception as e:
            print(f"Erro ao carregar dados: {e}")
            self.t, self.u, self.y = None, None, None
            self.k, self.tau, self.theta = np.nan, np.nan, np.nan
            return False

    def run_identification(self):
        """
        Calcula os parâmetros FOPDT pelos métodos de Smith e Sundaresan, 
        selecionando o par (tau, theta) com o menor RMSE (EQM).
        """
        if self.t is None or not np.isfinite(self.k):
            return None

        # --- 1. Método de Smith ---
        t1_smith, t2_smith = self._time_at_norm(0.283), self._time_at_norm(0.632)
        if np.isfinite(t1_smith) and np.isfinite(t2_smith):
            tau_smith = 1.5 * (t2_smith - t1_smith)
            theta_smith = t2_smith - tau_smith
        else:
            tau_smith, theta_smith = np.nan, np.nan
            
        y_hat_smith = self._simulate_fopdt(self.k, tau_smith, theta_smith, self.t, self.y0, self.du)
        rmse_smith = self._rmse(self.y, y_hat_smith)
        
        # --- 2. Método de Sundaresan & Krishnaswamy ---
        t1_sun, t2_sun = self._time_at_norm(0.353), self._time_at_norm(0.853)
        if np.isfinite(t1_sun) and np.isfinite(t2_sun):
            tau_sun = (2/3) * (t2_sun - t1_sun)
            theta_sun = 1.3 * t1_sun - 0.29 * t2_sun
        else:
            tau_sun, theta_sun = np.nan, np.nan
            
        y_hat_sun = self._simulate_fopdt(self.k, tau_sun, theta_sun, self.t, self.y0, self.du)
        rmse_sun = self._rmse(self.y, y_hat_sun)

        # --- 3. Seleção do Modelo ---
        # Escolhe o modelo com o menor erro (RMSE)
        if rmse_smith <= rmse_sun:
            self.tau, self.theta, self.method_id = tau_smith, theta_smith, "Smith"
            best_rmse = rmse_smith
        else:
            self.tau, self.theta, self.method_id = tau_sun, theta_sun, "Sundaresan"
            best_rmse = rmse_sun

        # Invalida o modelo se os parâmetros finais não atenderem às condições (tau>0, theta>=0, finitos)
        if not (np.isfinite(self.tau) and self.tau > 0 and np.isfinite(self.theta) and self.theta >= 0):
             self.tau, self.theta, self.method_id = np.nan, np.nan, "Inválido"
             best_rmse = np.inf

        y_model_final = self._simulate_fopdt(self.k, self.tau, self.theta, self.t, self.y0, self.du)

        return {
            'k': self.k,
            'tau': self.tau,
            'theta': self.theta,
            'rmse': best_rmse,
            'method_id': self.method_id,
            'y_exp': self.y,
            't_exp': self.t,
            'y_model': y_model_final
        }
        
    def calculate_pid_tuning(self, method, lambda_val=None):
        """Calcula os parâmetros Kp, Ti, Td com base na regra de sintonia FOPDT selecionada."""
        
        # Pré-condição: Modelo FOPDT deve ser válido
        if not (np.isfinite(self.k) and self.tau > 0 and self.theta >= 0 and self.k != 0):
             self.Kp, self.Ti, self.Td = 0.0, 0.0, 0.0
             return 0.0, 0.0, 0.0

        k, tau, theta = self.k, self.tau, self.theta
        Kp, Ti, Td = np.nan, np.nan, np.nan
        
        # --- 1. Ziegler-Nichols Malha Aberta ---
        if method == "Ziegler-Nichols MA":
            Kp = (1.2 * tau) / (k * theta)
            Ti = 2 * theta
            Td = theta / 2
        
        # --- 2. Método do Modelo Interno (IMC) ---
        elif method == "IMC":
            lambda_val = lambda_val if lambda_val is not None and lambda_val > 0 else 1.0 
            Kp = (2 * tau + theta) / (k * (2 * lambda_val + theta))
            Ti = tau + theta / 2
            Td = (tau * theta) / (2 * tau + theta)
        
        # --- 3. Método CHR (Chien, Hrones e Reswick) ---
        elif method == "CHR sem Sobressinal":
            Kp = (0.6 * tau) / (k * theta)
            Ti = tau
            Td = theta / 2
            
        elif method == "CHR com Sobressinal":
            # Projetado para 20% de overshoot
            Kp = (0.95 * tau) / (k * theta)
            Ti = 1.357 * tau
            Td = 0.473 * theta

        # --- 4. Método ITAE (REQUISITO GRUPO 3) ---
        elif method == "ITAE":
            # Usa correlações para minimizar o erro na resposta transitória
            A, B, C, D, E, F = self.ITAE_CONST
            ratio = theta / tau
            Kp = (A / k) * (ratio)**B
            Ti = tau * (C + D * ratio)
            Td = tau * E * (ratio)**F

        # --- 5. Método Cohen e Coon (CC) ---
        elif method == "Cohen e Coon":
            # Kp = (1/k) * (16*tau + 3*theta) / (12*theta)
            ratio = theta / tau
            Kp = (1 / k) * (16 * tau + 3 * theta) / (12 * theta)
            # Ti = theta * (32 + 6*ratio) / (13 + 8*ratio)
            Ti = theta * (32 + 6 * ratio) / (13 + 8 * ratio)
            # Td = 4*theta / (11 + 2*ratio)
            Td = 4 * theta / (11 + 2 * ratio)

        # Armazena e retorna os valores IAL
        self.Kp, self.Ti, self.Td = Kp, Ti, Td
        
        return Kp, Ti, Td


    def simulate_closed_loop(self, setpoint=1.0, k_p=None, t_i=None, t_d=None):
        """
        Executa a simulação do sistema em malha fechada T(s) com os parâmetros PID fornecidos.
        """
        if self.t is None or not np.isfinite(self.tau) or not np.isfinite(self.k):
            return None, None, None
            
        # Leitura dos parâmetros PID da IHM (ou usa os calculados se forem válidos)
        Kp = k_p if k_p is not None and np.isfinite(k_p) else self.Kp
        Ti = t_i if t_i is not None and t_i > 1e-6 and np.isfinite(t_i) else 1e-6 
        Td = t_d if t_d is not None and np.isfinite(t_d) else self.Td
        
        self.Kp, self.Ti, self.Td = Kp, Ti, Td 

        # 1. Função de transferência do Controlador Gc(s)
        Gc = self._get_pid_tf()

        # 2. Função de transferência da Planta Gp(s)
        G_process = tf([self.k], [self.tau, 1])
        nd, dd = pade(self.theta, 1) 
        Gd = tf(nd, dd)
        Gp = G_process * Gd
        
        # 3. Função de transferência do Sistema em Malha Fechada T(s) = Gc*Gp / (1 + Gc*Gp)
        T = feedback(Gc * Gp, 1)

        # 4. Simulação da Resposta ao Degrau e escalamento pelo SetPoint
        t_sim, y_cl = step_response(T, T=self.t)
        y_cl *= setpoint

        # 5. Cálculo das Métricas de Desempenho
        metrics_data = self.calculate_metrics(t_sim, y_cl, setpoint)
        
        return t_sim, y_cl, metrics_data
        
    def calculate_metrics(self, t, y, y_final):
        """
        Calcula as métricas de desempenho (tr, ts, Mp, ess) da resposta ao degrau.
        """
        if y is None or y_final is None or len(y) == 0 or not np.any(np.isfinite(y)):
            return {'tr': np.nan, 'ts': np.nan, 'Mp': np.nan, 'ess': np.nan}

        # 1. Erro em Regime Permanente (ess)
        yss = y[-1]
        ess = abs(y_final - yss)

        # 2. Tempo de Subida (tr): 10% a 90%
        t10 = t[np.where(y >= 0.1 * y_final)[0][0]] if np.any(y >= 0.1 * y_final) else np.nan
        t90 = t[np.where(y >= 0.9 * y_final)[0][0]] if np.any(y >= 0.9 * y_final) else np.nan
        tr = t90 - t10 if np.isfinite(t10) and np.isfinite(t90) else np.nan

        # 3. Overshoot (Mp %):
        Mp_value = np.max(y)
        Mp = (Mp_value - y_final) / y_final * 100.0 if y_final != 0 and Mp_value > y_final else 0.0
        
        # 4. Tempo de Acomodação (ts): ±2%
        banda = 0.02 * y_final
        ts = np.nan
        
        # Busca o primeiro instante onde a resposta entra e permanece dentro da banda de 2%
        idx_in_band = np.where(np.abs(y - y_final) <= banda)[0]
        
        if len(idx_in_band) > 0:
            for i in idx_in_band:
                if np.all(np.abs(y[i:] - y_final) <= banda):
                    ts = t[i]
                    break
        
        return {
            'tr': tr,
            'ts': ts,
            'Mp': Mp,
            'ess': ess
        }