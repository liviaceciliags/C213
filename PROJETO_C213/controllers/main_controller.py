from PyQt5.QtWidgets import QFileDialog, QMessageBox
from PyQt5.QtCore import QLocale, Qt
import numpy as np
import pyqtgraph as pg
import pyqtgraph.exporters 

# Configura o locale para ponto decimal (necessário para comunicação numérica precisa)
QLocale.setDefault(QLocale(QLocale.English, QLocale.AnyCountry))

class MainController:
    """
    Controlador da Aplicação (Camada C - Controller).
    
    Gerencia o fluxo de trabalho do usuário, recebendo comandos da Interface (View) 
    e orquestrando a execução da lógica de cálculo (Model).
    """
    
    def __init__(self, model, view):
        self.model = model  # Acesso aos métodos de cálculo (ID, Sintonia, Simulação)
        self.view = view    # Acesso aos widgets da interface (IHM)
        self.last_plot_data = None 
        
        # O processo inicia conectando a interface ao backend
        self.connect_signals()

    def connect_signals(self):
        """Conecta eventos (Sinais) dos widgets aos métodos de processamento (Slots) do Controller."""
        
        # --- Aba de Identificação: Fluxo de Modelagem ---
        self.view.identification_tab.btn_load.clicked.connect(self.load_data_action)
        self.view.identification_tab.btn_identify.clicked.connect(self.run_identification_action)
        self.view.identification_tab.btn_export_graph.clicked.connect(
            # Usa lambda para passar o widget de plotagem como argumento para a função genérica de exportação
            lambda: self.export_graph_action(self.view.identification_tab.plot_widget)
        )
        
        # --- Aba de Controle PID: Fluxo de Sintonia e Simulação ---
        self.view.control_tab.btn_tune_simulate.clicked.connect(self.run_tuning_simulation_action)
        self.view.control_tab.btn_export_graph.clicked.connect(
            lambda: self.export_graph_action(self.view.control_tab.plot_widget)
        )
        self.view.control_tab.btn_clear_manual.clicked.connect(self.view.control_tab.clear_tuning_fields)
        
        # Lógica de interface para alternância de modos
        self.view.control_tab.radio_method.toggled.connect(self.toggle_tuning_mode)
        self.view.control_tab.cb_tuning_method.currentTextChanged.connect(self.handle_method_change)
        
        # Reseta as métricas ao mudar o SetPoint para forçar uma nova simulação
        self.view.control_tab.le_setpoint.valueChanged.connect(self.view.control_tab.clear_metrics)
        
        # Configuração inicial para garantir o estado correto da IHM
        self.toggle_tuning_mode(self.view.control_tab.radio_method.isChecked())
        self.handle_method_change(self.view.control_tab.cb_tuning_method.currentText())


    # -------------------------------------------------------------------------
    # --- Métodos de Identificação de Sistemas ---
    # -------------------------------------------------------------------------

    def load_data_action(self):
        """Abre o explorador de arquivos, carrega o dataset e exibe a curva de reação bruta."""
        
        # Reseta o estado da aplicação antes de carregar
        self.view.identification_tab.clear_results()
        self.view.control_tab.setEnabled(False) 
        self.view.control_tab.clear_metrics()
        
        filepath, _ = QFileDialog.getOpenFileName(
            self.view, "Carregar Dados Experimentais", "", "MATLAB Files (*.mat)"
        )
        
        if filepath:
            if self.model.load_data(filepath):
                self.view.identification_tab.status_label.setText(f"Status: Dados '{filepath.split('/')[-1]}' carregados.")
                self.view.identification_tab.btn_identify.setEnabled(True)
                
                # Plotagem inicial: mostra apenas os dados brutos (curva de reação)
                self.plot_identification_data(self.model.t, self.model.y, clear_model=True)
            else:
                QMessageBox.critical(self.view, "Erro de Carregamento", "Falha ao carregar o arquivo .mat ou dados inválidos.")

    def run_identification_action(self):
        """Executa a identificação FOPDT, seleciona o melhor modelo (menor RMSE) e atualiza a IHM."""
        
        self.view.control_tab.clear_metrics()
        self.view.control_tab.plot_widget.clear()
        self.view.identification_tab.btn_export_graph.setEnabled(False)

        # Chama a lógica de cálculo do Modelo
        results = self.model.run_identification()
        
        if results and np.isfinite(results['k']) and np.isfinite(results['tau']):
            # 1. Exibe os parâmetros FOPDT do modelo escolhido
            self.view.identification_tab.le_k.setText(f"{results['k']:.4f}")
            self.view.identification_tab.le_tau.setText(f"{results['tau']:.4f}")
            self.view.identification_tab.le_theta.setText(f"{results['theta']:.4f}")
            self.view.identification_tab.le_rmse.setText(f"{results['rmse']:.4e}") 
            self.view.identification_tab.le_method.setText(results['method_id'])

            # 2. Plota o modelo FOPDT sobreposto à curva experimental
            self.plot_identification_data(
                results['t_exp'], results['y_exp'], 
                results['t_exp'], results['y_model'], 
                results['method_id'],
                clear_model=False
            )

            # 3. Prepara a aba de Controle para a próxima etapa
            self.view.control_tab.setEnabled(True)
            self.view.identification_tab.btn_export_graph.setEnabled(True)
            
            # Inicializa SetPoint com o valor de regime (y_final)
            self.view.control_tab.le_setpoint.setValue(self.model.y0 + self.model.den_norm) 
            
            # Força o cálculo inicial da sintonia (para exibir os Kp, Ti, Td iniciais)
            self.run_tuning_calculation_action()
            
        else:
             QMessageBox.critical(self.view, "Erro de Identificação", "Os dados não permitiram um modelo FOPDT válido (k, tau ou theta inválido).")
             self.view.control_tab.setEnabled(False)
             self.view.identification_tab.clear_results()


    def plot_identification_data(self, t_exp, y_exp, t_model=None, y_model=None, method_id="", clear_model=False):
        """Função genérica de plotagem para a aba de Identificação."""
        plot = self.view.identification_tab.plot_widget.plotItem
        
        plot.clear()
        
        # Curva Experimental (Dados da Planta)
        plot.plot(t_exp, y_exp, pen='k', name='Experimental')
        
        # Curva do Modelo FOPDT
        if y_model is not None and not np.any(np.isnan(y_model)):
            plot.plot(t_model, y_model, pen=pg.mkPen('r', width=2), name=f'Modelo FOPDT ({method_id})')
        
        # Atualiza o título do gráfico
        if not clear_model:
             plot.setTitle(f"Curva de Reação - Modelo FOPDT: {method_id}")
        else:
             plot.setTitle("Curva de Reação - Dados Experimentais")

        plot.addLegend()


    # -------------------------------------------------------------------------
    # --- Métodos de Sintonia e Simulação (Controle PID) ---
    # -------------------------------------------------------------------------

    def toggle_tuning_mode(self, checked):
        """Alterna a IHM entre os modos de Sintonia Automática (Método) e Manual."""
        is_method_mode = self.view.control_tab.radio_method.isChecked()
        is_manual_mode = not is_method_mode
        
        # Controla a edição dos parâmetros e a visibilidade dos controles
        self.view.control_tab.cb_tuning_method.setEnabled(is_method_mode)
        self.view.control_tab.le_kp.setReadOnly(is_method_mode)
        self.view.control_tab.le_ti.setReadOnly(is_method_mode)
        self.view.control_tab.le_td.setReadOnly(is_method_mode)
        self.view.control_tab.btn_clear_manual.setEnabled(is_manual_mode)

        self.view.control_tab.clear_metrics()
        self.view.control_tab.plot_widget.clear()

        # Atualiza os campos e recalcula se necessário
        self.handle_method_change(self.view.control_tab.cb_tuning_method.currentText())
        if is_method_mode:
            self.run_tuning_calculation_action()
        else:
            self.view.control_tab.clear_tuning_fields() # Prepara para a inserção manual


    def handle_method_change(self, method):
        """Gerencia o estado do campo Lambda (λ) do IMC e atualiza os parâmetros PID."""
        is_imc = (method == "IMC")
        
        # Lambda é editável apenas se for IMC E o modo 'Método' estiver ativo
        lambda_enabled = is_imc and self.view.control_tab.radio_method.isChecked()
        
        self.view.control_tab.le_lambda.setEnabled(lambda_enabled)
        self.view.control_tab.le_lambda.setReadOnly(not lambda_enabled)
        
        # Recalcula o Kp/Ti/Td se o método for alterado no modo automático
        if self.view.control_tab.radio_method.isChecked():
            self.run_tuning_calculation_action()


    def run_tuning_calculation_action(self):
        """Calcula os parâmetros Kp, Ti, Td usando o método de sintonia selecionado e preenche a IHM."""
        method = self.view.control_tab.cb_tuning_method.currentText()
        lambda_val = self.view.control_tab.le_lambda.value() if method == "IMC" else None
        
        # Chama o Model para obter a sintonia
        Kp, Ti, Td = self.model.calculate_pid_tuning(method, lambda_val)
        
        # Atualiza os spin boxes com os valores calculados
        if np.isfinite(Kp):
            self.view.control_tab.le_kp.setValue(Kp)
            self.view.control_tab.le_ti.setValue(Ti)
            self.view.control_tab.le_td.setValue(Td)
        else:
            # Em caso de falha no cálculo (ex: Kp = NaN)
            self.view.control_tab.le_kp.setValue(0.0)
            self.view.control_tab.le_ti.setValue(0.01)
            self.view.control_tab.le_td.setValue(0.0)
            if self.view.control_tab.radio_method.isChecked():
                QMessageBox.warning(self.view, "Aviso", "Parâmetros FOPDT inválidos. A sintonia automática não pode ser calculada.")


    def run_tuning_simulation_action(self):
        """Executa a simulação em malha fechada e exibe o desempenho do controlador."""
        
        setpoint = self.view.control_tab.le_setpoint.value()
        
        if not np.isfinite(self.model.k) or not np.isfinite(self.model.tau):
            QMessageBox.warning(self.view, "Aviso", "Identifique o modelo FOPDT na primeira aba antes de simular.")
            return

        # Lê os parâmetros do controlador diretamente da IHM (garante o uso de valores manuais, se aplicável)
        Kp = self.view.control_tab.le_kp.value()
        Ti = self.view.control_tab.le_ti.value()
        Td = self.view.control_tab.le_td.value()
        
        # Chama a simulação do sistema controlado
        t_sim, y_cl, metrics = self.model.simulate_closed_loop(setpoint, Kp, Ti, Td)
        
        self.view.control_tab.clear_metrics() 

        # 3. Atualiza a interface com os resultados
        if y_cl is not None and not np.any(np.isnan(y_cl)):
            # Atualiza Gráfico
            plot = self.view.control_tab.plot_widget.plotItem
            plot.clear()
            plot.plot(self.model.t, y_cl, pen=pg.mkPen('k', width=2), name='Resposta PID')
            
            # Linha de SetPoint (Horizontal, cor azul)
            setpoint_line = pg.InfiniteLine(pos=setpoint, angle=0, pen=pg.mkPen('b', width=1, style=Qt.DashLine), name='SetPoint')
            plot.addItem(setpoint_line)
            
            # Linha de Overshoot (Mp)
            Mp_value = setpoint * (1 + metrics['Mp'] / 100.0)
            
            # Plota a linha de overshoot (vermelha pontilhada)
            if metrics['Mp'] > 0.01: 
                overshoot_line = pg.InfiniteLine(pos=Mp_value, angle=0, pen=pg.mkPen('r', width=1, style=Qt.DotLine), name=f'Mp ({metrics["Mp"]:.2f}%)')
                plot.addItem(overshoot_line)
            
            plot.addLegend()
            plot.setTitle(f"Resposta PID (Kp={Kp:.3g}, Ti={Ti:.3g}, Td={Td:.3g})")
            
            # Habilita exportação e atualiza métricas
            self.view.control_tab.btn_export_graph.setEnabled(True)
            self.view.control_tab.le_tr.setText(f"{metrics['tr']:.3f} s")
            self.view.control_tab.le_ts.setText(f"{metrics['ts']:.3f} s")
            self.view.control_tab.le_mp.setText(f"{metrics['Mp']:.2f} %")
            self.view.control_tab.le_ess.setText(f"{metrics['ess']:.3f}")
        else:
            self.view.control_tab.btn_export_graph.setEnabled(False)
            QMessageBox.critical(self.view, "Erro de Simulação", "O controlador é instável ou os parâmetros (Ti, k, tau) são inválidos.")

    # -------------------------------------------------------------------------
    # --- Ações de Exportação ---
    # -------------------------------------------------------------------------
    
    def export_graph_action(self, plot_widget):
        """Exporta o gráfico atual (PyQtGraph) para um arquivo de imagem (PNG/JPG)."""
        # 1. Abrir diálogo de salvamento
        filepath, _ = QFileDialog.getSaveFileName(
            self.view, "Salvar Gráfico", "resposta_pid", "PNG Image (*.png);;JPEG Image (*.jpg)"
        )
        
        if filepath:
            try:
                # Usa o submódulo pyqtgraph.exporters para renderizar a imagem
                exporter = pg.exporters.ImageExporter(plot_widget.plotItem)
                exporter.export(filepath)
                QMessageBox.information(self.view, "Sucesso", f"Gráfico salvo em: {filepath}")
            except Exception as e:
                QMessageBox.critical(self.view, "Erro de Exportação", f"Falha ao salvar o gráfico: {e}")