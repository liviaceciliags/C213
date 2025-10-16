import numpy as np
import scipy.io as sio
from control import tf, feedback, step_response, pade

class PIDModel:
    """
    Model (Camada M - Modelo).
    Carrega dados, identifica FOPDT, calcula sintonias e simula respostas.
    - Identificação: gráficos ABSOLUTOS (começam em y0 e vão a y0+dy)
    - Controle PID (CHR/ITAE): resposta RELATIVA (Δy), com SP ABSOLUTO convertido p/ Δ
    """

    def __init__(self, pade_id: int = 10, pade_cl: int = 1):
        # Parâmetros FOPDT
        self.k, self.tau, self.theta = np.nan, np.nan, np.nan

        # Dados experimentais e derivados
        self.t, self.u, self.y = None, None, None
        self.y0, self.du = np.nan, np.nan      # nível inicial e degrau da entrada
        self.den_norm = np.nan                 # dy = y1 - y0
        self.method_id = "N/A"

        # Ordem do Padé (identificação/aberta e fechada)
        self.pade_order_id = pade_id           # identificação/malha aberta (10)
        self.pade_order_cl = pade_cl           # malha fechada (1)

        # Últimos parâmetros PID
        self.Kp, self.Ti, self.Td = 0.0, 0.0, 0.0

        # Constantes ITAE
        self.ITAE_CONST = [0.965, -0.85, 0.796, -0.147, 0.308, 0.929]

    # -------------------------------------------------------------------------
    # Utilidades
    # -------------------------------------------------------------------------
    def _time_at_norm(self, percent: float):
        """Retorna o tempo em que (y - y0)/(y1 - y0) atinge 'percent'."""
        if self.t is None or not np.isfinite(self.den_norm) or abs(self.den_norm) < 1e-9:
            return np.nan
        y_norm = (self.y - self.y0) / self.den_norm
        idx = np.where(y_norm >= percent)[0]
        return self.t[idx[0]] if len(idx) else np.nan

    def _simulate_fopdt(self, k, tau, theta, t_data, y0):
        """
        Simula Gp(s) = (k/(tau*s+1))*e^{-theta s} para passo unitário.
        Retorna saída ABSOLUTA com DC correto: y0 + dy*(y_step/k).
        """
        if not (np.isfinite(k) and np.isfinite(tau) and np.isfinite(theta)) or tau <= 0 or theta < 0:
            return np.full_like(t_data, np.nan, dtype=float)

        G = tf([k], [tau, 1])
        nd, dd = pade(theta, self.pade_order_id)            # Padé da IDENTIFICAÇÃO (10)
        Gp = G * tf(nd, dd)

        _, y_step = step_response(Gp, T=t_data)             # passo unitário → final = k
        if abs(k) < 1e-12 or not np.isfinite(self.den_norm):
            return np.full_like(t_data, np.nan, dtype=float)

        # Força platô igual ao experimental: y0 + dy
        return y0 + (self.den_norm) * (y_step / k)

    def _rmse(self, y_true, y_hat):
        if y_hat is None or np.isnan(y_hat).any() or len(y_true) != len(y_hat):
            return np.inf
        e = y_true - y_hat
        return np.sqrt(np.mean(e**2))

    def _get_pid_tf(self):
        """PID paralelo: (Kp*Td*s^2 + Kp*s + Kp/Ti) / s"""
        Kp, Ti, Td = self.Kp, self.Ti, self.Td
        if (not np.isfinite(Kp)) or (not np.isfinite(Ti)) or (not np.isfinite(Td)):
            return tf([0], [1])
        if Ti <= 0 or (Kp == 0.0 and Ti == 0.0 and Td == 0.0):
            return tf([0], [1])
        num = [Kp * Td, Kp, Kp / Ti]
        den = [1, 0]
        return tf(num, den)

    # -------------------------------------------------------------------------
    # Carregamento e Identificação
    # -------------------------------------------------------------------------
    def load_data(self, filepath):
        """
        Carrega t,u,y do .mat com busca robusta de chaves e limpeza básica.
        Detecta degrau em u, calcula du, y0, dy, k. Sem fallback em mean(u).
        Retorna True se k válido; False caso contrário.
        """
        def _as_1d(a):
            a = np.asarray(a)
            a = np.ravel(a)  # 1D
            return a.astype(float)

        def _find_first_key(dct, aliases):
            for k in dct.keys():
                lk = k.lower()
                for a in aliases:
                    if lk == a or lk.endswith(a):  # aceita 'dados/tiempo' etc.
                        return k
            return None

        try:
            data = sio.loadmat(filepath)
        except NotImplementedError:
            # v7.3 (HDF5)
            try:
                import h5py
                with h5py.File(filepath, "r") as f:
                    data = {k: f[k][()] for k in f.keys()}
            except Exception as e:
                print(f"Erro ao ler .mat (v7.3?): {e}")
                self.t = self.u = self.y = None
                self.k = self.tau = self.theta = np.nan
                return False
        except Exception as e:
            print(f"Erro ao ler .mat: {e}")
            self.t = self.u = self.y = None
            self.k = self.tau = self.theta = np.nan
            return False

        # Localiza chaves
        t_key = _find_first_key(data, ["tiempo", "tempo", "time", "t"])
        u_key = _find_first_key(data, ["entrada", "input", "u"])
        y_key = _find_first_key(data, ["salida", "output", "y"])

        if t_key is None or u_key is None or y_key is None:
            print(f"Chaves não encontradas. Achei: t={t_key}, u={u_key}, y={y_key}.")
            self.t = self.u = self.y = None
            self.k = self.tau = self.theta = np.nan
            return False

        # 1D e limpeza
        t = _as_1d(data[t_key])
        u = _as_1d(data[u_key])
        y = _as_1d(data[y_key])

        n = min(len(t), len(u), len(y))
        t, u, y = t[:n], u[:n], y[:n]

        mask = np.isfinite(t) & np.isfinite(u) & np.isfinite(y)
        t, u, y = t[mask], u[mask], y[mask]

        if len(t) < 3:
            print("Dados insuficientes após limpeza.")
            self.t = self.u = self.y = None
            self.k = self.tau = self.theta = np.nan
            return False

        # Tempo crescente e sem duplicados
        sort_idx = np.argsort(t)
        t, u, y = t[sort_idx], u[sort_idx], y[sort_idx]
        uniq_mask = np.concatenate(([True], np.diff(t) > 0))
        t, u, y = t[uniq_mask], u[uniq_mask], y[uniq_mask]

        # Degrau em u: maior |Δu|
        du_vec = np.diff(u)
        if len(du_vec) == 0:
            print("Sinal de entrada muito curto para detectar degrau.")
            self.t = self.u = self.y = None
            self.k = self.tau = self.theta = np.nan
            return False

        step_idx = int(np.argmax(np.abs(du_vec)))
        w_pre, w_post = 25, 25

        u0 = float(np.median(u[max(0, step_idx - w_pre): step_idx])) if step_idx > 0 else float(np.median(u[:w_pre]))
        u1 = float(np.median(u[step_idx + 1: min(len(u), step_idx + 1 + w_post)])) if step_idx + 1 < len(u) else float(np.median(u[-w_post:]))

        self.y0 = float(np.median(y[max(0, step_idx - w_pre): step_idx])) if step_idx > 0 else float(np.median(y[:w_pre]))
        y1      = float(np.median(y[-w_post:]))

        self.du       = u1 - u0
        self.den_norm = y1 - self.y0  # dy

        # Janela maior se du pequeno
        eps = 1e-6
        if abs(self.du) < eps:
            w_pre2, w_post2 = 50, 50
            u0b = float(np.median(u[max(0, step_idx - w_pre2): step_idx])) if step_idx > 0 else u0
            u1b = float(np.median(u[step_idx + 1: min(len(u), step_idx + 1 + w_post2)])) if step_idx + 1 < len(u) else u1
            self.du = u1b - u0b

        if abs(self.du) < eps or abs(self.den_norm) < eps:
            print(f"Degrau inválido: du={self.du:.3e}, dy={self.den_norm:.3e}.")
            self.t = t; self.u = u; self.y = y
            self.k = np.nan
            return False

        self.t, self.u, self.y = t, u, y
        self.k = self.den_norm / self.du
        return np.isfinite(self.k)

    def run_identification(self):
        """
        Estima (tau, theta) por Smith e Sundaresan & Krishnaswamy; escolhe por RMSE.
        Fallback: modelo sem atraso (theta=0) usando 10–90%.
        """
        if self.t is None or not np.isfinite(self.k):
            return None

        # Smith (28.3% e 63.2%)
        t1s, t2s = self._time_at_norm(0.283), self._time_at_norm(0.632)
        if np.isfinite(t1s) and np.isfinite(t2s) and t2s > t1s:
            tau_smith  = 1.5 * (t2s - t1s)
            theta_smith = t2s - tau_smith
        else:
            tau_smith, theta_smith = np.nan, np.nan

        y_hat_smith = self._simulate_fopdt(self.k, tau_smith, theta_smith, self.t, self.y0)
        rmse_smith  = self._rmse(self.y, y_hat_smith)

        # Sundaresan & Krishnaswamy (35.3% e 85.3%)
        t1u, t2u = self._time_at_norm(0.353), self._time_at_norm(0.853)
        if np.isfinite(t1u) and np.isfinite(t2u) and t2u > t1u:
            tau_sun   = (2/3) * (t2u - t1u)
            theta_sun = 1.3 * t1u - 0.29 * t2u
        else:
            tau_sun, theta_sun = np.nan, np.nan

        y_hat_sun = self._simulate_fopdt(self.k, tau_sun, theta_sun, self.t, self.y0)
        rmse_sun  = self._rmse(self.y, y_hat_sun)

        candidates = []
        for name, tau_c, th_c, yhat_c, rmse_c in [
            ("Smith", tau_smith, theta_smith, y_hat_smith, rmse_smith),
            ("Sundaresan", tau_sun, theta_sun, y_hat_sun, rmse_sun),
        ]:
            if np.isfinite(tau_c) and tau_c > 0 and np.isfinite(th_c) and th_c >= 0:
                candidates.append((rmse_c, name, tau_c, th_c, yhat_c))

        # Fallback sem atraso (theta=0) com tau por 10–90%
        if not candidates:
            t10, t90 = self._time_at_norm(0.10), self._time_at_norm(0.90)
            if np.isfinite(t10) and np.isfinite(t90) and t90 > t10:
                tau0, theta0 = (t90 - t10) / 2.2, 0.0
                yhat0 = self._simulate_fopdt(self.k, tau0, theta0, self.t, self.y0)
                rmse0 = self._rmse(self.y, yhat0)
                candidates.append((rmse0, "SemAtraso(backup)", tau0, theta0, yhat0))
            else:
                return None

        rmse, name, tau, theta, y_model_final = min(candidates, key=lambda x: x[0])
        self.tau, self.theta, self.method_id = tau, theta, name

        return {
            'k': self.k,
            'tau': self.tau,
            'theta': self.theta,
            'rmse': rmse,
            'method_id': self.method_id,
            'y_exp': self.y,
            't_exp': self.t,
            'y_model': y_model_final
        }

    # -------------------------------------------------------------------------
    # Sintonias
    # -------------------------------------------------------------------------
    def calculate_pid_tuning(self, method, lambda_val=None):
        """Calcula Kp, Ti, Td conforme o método informado."""
        if not (np.isfinite(self.k) and self.tau > 0 and self.theta >= 0 and self.k != 0):
            self.Kp, self.Ti, self.Td = 0.0, 0.0, 0.0
            return 0.0, 0.0, 0.0

        k, tau, theta = self.k, self.tau, self.theta
        Kp, Ti, Td = np.nan, np.nan, np.nan

        if method == "Ziegler-Nichols MA":
            Kp = (1.2 * tau) / (k * theta); Ti = 2 * theta; Td = theta / 2

        elif method == "IMC":
            lam = lambda_val if (lambda_val is not None and lambda_val > 0) else 1.0
            Kp = (2 * tau + theta) / (k * (2 * lam + theta))
            Ti = tau + theta / 2
            Td = (tau * theta) / (2 * tau + theta)

        elif method == "CHR sem Sobressinal":
            Kp = (0.6 * tau) / (k * theta); Ti = tau; Td = theta / 2

        elif method == "CHR com Sobressinal":
            Kp = (0.95 * tau) / (k * theta); Ti = 1.357 * tau; Td = 0.473 * theta

        elif method == "ITAE":
            A, B, C, D, E, F = self.ITAE_CONST
            r = theta / tau
            Kp = (A / k) * (r)**B
            Ti = tau * (C + D * r)
            Td = tau * E * (r)**F

        elif method == "Cohen e Coon":
            r = theta / tau
            Kp = (1 / k) * (16 * tau + 3 * theta) / (12 * theta)
            Ti = theta * (32 + 6 * r) / (13 + 8 * r)
            Td = 4 * theta / (11 + 2 * r)

        self.Kp, self.Ti, self.Td = Kp, Ti, Td
        return Kp, Ti, Td

    # -------------------------------------------------------------------------
    # Simulações (Controle PID - CHR/ITAE em Δy com SP absoluto convertido)
    # -------------------------------------------------------------------------
    def simulate_closed_loop(self, setpoint=1.0, k_p=None, t_i=None, t_d=None):
        """
        Simula T(s) = (Gc*Gp)/(1+Gc*Gp) para a aba Controle PID.
        Retorna **resposta relativa (Δy)**: 0 → (SP_abs - y0).
        O campo SP na GUI é ABSOLUTO; aqui convertemos para Δ.
        """
        if self.t is None or not np.isfinite(self.tau) or not np.isfinite(self.k):
            return None, None, None

        # Parâmetros PID (entrada da GUI ou últimos calculados)
        Kp = k_p if (k_p is not None and np.isfinite(k_p)) else self.Kp
        Ti = t_i if (t_i is not None and t_i > 1e-6 and np.isfinite(t_i)) else 1e-6
        Td = t_d if (t_d is not None and np.isfinite(t_d)) else self.Td
        self.Kp, self.Ti, self.Td = Kp, Ti, Td

        # Controlador e planta (Padé de malha fechada = 1, como no Colab)
        Gc = self._get_pid_tf()
        nd, dd = pade(self.theta, self.pade_order_cl)       # 1
        Gp = tf([self.k], [self.tau, 1]) * tf(nd, dd)
        T = feedback(Gc * Gp, 1)

        # Resposta unitária 0→1
        t_sim, y_unit = step_response(T, T=self.t)

        # Converte SP ABSOLUTO da GUI para Δ-alvo
        if setpoint is None or not np.isfinite(setpoint):
            setpoint = self.y0 + self.den_norm              # default = final experimental

        y_final_rel = (setpoint - self.y0)                  # Δ-alvo
        y_cl_rel    = y_final_rel * y_unit                  # 0 → (SP - y0)

        # Métricas em coordenadas relativas
        metrics_data = self.calculate_metrics(t_sim, y_cl_rel, y_final_rel)
        return t_sim, y_cl_rel, metrics_data

    # -------------------------------------------------------------------------
    # Métricas
    # -------------------------------------------------------------------------
    def calculate_metrics(self, t, y, y_final):
        """
        Calcula tr (10–90%), ts (±2%), Mp (%) e ess, assumindo referência = y_final.
        Funciona tanto para relativo (y_final=Δy) quanto para absoluto (y_final=valor final).
        """
        if y is None or y_final is None or len(y) == 0 or not np.any(np.isfinite(y)):
            return {'tr': np.nan, 'ts': np.nan, 'Mp': np.nan, 'ess': np.nan}

        yss = y[-1]
        ess = abs(y_final - yss)

        t10 = t[np.where(y >= 0.1 * y_final)[0][0]] if np.any(y >= 0.1 * y_final) else np.nan
        t90 = t[np.where(y >= 0.9 * y_final)[0][0]] if np.any(y >= 0.9 * y_final) else np.nan
        tr = t90 - t10 if np.isfinite(t10) and np.isfinite(t90) else np.nan

        Mp_value = np.max(y)
        Mp = (Mp_value - y_final) / y_final * 100.0 if (y_final != 0 and Mp_value > y_final) else 0.0

        band = 0.02 * abs(y_final)
        outside = np.where(np.abs(y - y_final) > band)[0]
        ts = (t[outside[-1] + 1] if len(outside) and outside[-1] + 1 < len(t) else (t[-1] if len(outside) else t[0]))

        return {'tr': tr, 'ts': ts, 'Mp': Mp, 'ess': ess}
