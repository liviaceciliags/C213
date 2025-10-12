from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel, QLineEdit, QGroupBox, QGridLayout, QComboBox, QRadioButton, QHBoxLayout, QDoubleSpinBox
from PyQt5.QtCore import QLocale, Qt
import pyqtgraph as pg
import numpy as np

class ControlTab(QWidget):
    """
    View (Camada V - Visão).
    Define a interface e os widgets interativos para a sintonia e simulação do Controlador PID.
    """
    
    TUNING_METHODS = [
        "Ziegler-Nichols MA", "IMC", "CHR sem Sobressinal", 
        "CHR com Sobressinal", "Cohen e Coon", "ITAE"
    ]
    
    def __init__(self):
        """Constrói a estrutura visual da aba de Controle PID."""
        super().__init__()
        self.layout = QVBoxLayout(self)
        
        # --- Grupo de Parâmetros de Sintonia ---
        tuning_group = QGroupBox("Seleção e Parâmetros de Sintonia")
        grid_tuning = QGridLayout(tuning_group)
        
        # Seleção de Modo (Rádio Buttons para Método/Manual)
        self.radio_method = QRadioButton("Método")
        self.radio_manual = QRadioButton("Manual")
        self.radio_method.setChecked(True) # Define o modo automático como padrão
        
        tuning_mode_layout = QHBoxLayout()
        tuning_mode_layout.addWidget(self.radio_method)
        tuning_mode_layout.addWidget(self.radio_manual)
        
        # ComboBox para seleção do método (Ziegler-Nichols, IMC, CHR, etc.)
        self.cb_tuning_method = QComboBox()
        self.cb_tuning_method.addItems(self.TUNING_METHODS)
        tuning_mode_layout.addWidget(self.cb_tuning_method)
        
        grid_tuning.addLayout(tuning_mode_layout, 0, 0, 1, 3)
        
        # Campos de entrada/saída para os parâmetros Kp, Ti, Td, Lambda (QDoubleSpinBox)
        self.le_kp = self._create_spin_box(decimals=4, max_val=1e6)
        self.le_ti = self._create_spin_box(decimals=4, min_val=1e-6, max_val=1e6) # Garante Ti > 0 para evitar singularidade
        self.le_td = self._create_spin_box(decimals=4, max_val=1e6)
        self.le_lambda = self._create_spin_box(decimals=4, min_val=0.01, max_val=1e3) # Lambda do IMC

        # Botão para limpar campos de PID (utilizado no modo Manual)
        self.btn_clear_manual = QPushButton("Limpar")
        self.btn_clear_manual.setToolTip("Limpar Kp, Ti, Td no modo Manual.") 
        self.btn_clear_manual.setFixedWidth(60)

        grid_tuning.addWidget(QLabel("Kp:"), 1, 0, alignment=Qt.AlignRight)
        grid_tuning.addWidget(self.le_kp, 1, 1)
        grid_tuning.addWidget(QLabel("Ti:"), 2, 0, alignment=Qt.AlignRight)
        grid_tuning.addWidget(self.le_ti, 2, 1)
        grid_tuning.addWidget(QLabel("Td:"), 3, 0, alignment=Qt.AlignRight)
        grid_tuning.addWidget(self.le_td, 3, 1)
        grid_tuning.addWidget(QLabel("λ (IMC):"), 4, 0, alignment=Qt.AlignRight)
        grid_tuning.addWidget(self.le_lambda, 4, 1)
        grid_tuning.addWidget(self.btn_clear_manual, 1, 2, 3, 1) # Botão Limpar posicionado ao lado dos parâmetros

        # --- Botões de Ação ---
        action_button_layout = QHBoxLayout()
        self.btn_tune_simulate = QPushButton("Sintonizar e Simular") 
        self.btn_export_graph = QPushButton("Exportar Gráfico") # Requisito de entrega
        self.btn_export_graph.setEnabled(False) 
        
        action_button_layout.addWidget(self.btn_tune_simulate)
        action_button_layout.addWidget(self.btn_export_graph)
        
        grid_tuning.addLayout(action_button_layout, 5, 0, 1, 3)
        self.layout.addWidget(tuning_group)
        
        # --- Grupo de Parâmetros de Controle e Métricas de Desempenho ---
        metrics_group = QGroupBox("Parâmetros de Controle e Métricas")
        grid_metrics = QGridLayout(metrics_group)
        
        # SetPoint (SP) - Valor de referência para a simulação
        self.le_setpoint = self._create_spin_box(min_val=-1e6, max_val=1e6, decimals=4, default_val=1.0)
        
        # Campos de saída (Métricas - somente leitura)
        self.le_tr = self._create_line_edit(initial_text="--- s")
        self.le_ts = self._create_line_edit(initial_text="--- s")
        self.le_mp = self._create_line_edit(initial_text="--- %")
        self.le_ess = self._create_line_edit(initial_text="---")

        grid_metrics.addWidget(QLabel("SetPoint (SP):"), 0, 0, alignment=Qt.AlignRight)
        grid_metrics.addWidget(self.le_setpoint, 0, 1)
        grid_metrics.addWidget(QLabel("Tempo de Subida (tr):"), 1, 0, alignment=Qt.AlignRight) 
        grid_metrics.addWidget(self.le_tr, 1, 1)
        grid_metrics.addWidget(QLabel("Tempo de Acomodação (ts):"), 2, 0, alignment=Qt.AlignRight) 
        grid_metrics.addWidget(self.le_ts, 2, 1)
        grid_metrics.addWidget(QLabel("Overshoot (Mp %):"), 3, 0, alignment=Qt.AlignRight) 
        grid_metrics.addWidget(self.le_mp, 3, 1)
        grid_metrics.addWidget(QLabel("Erro em Regime (ess):"), 4, 0, alignment=Qt.AlignRight)
        grid_metrics.addWidget(self.le_ess, 4, 1)

        self.layout.addWidget(metrics_group)

        # --- Widget do Gráfico ---
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w') 
        self.plot_widget.setLabel('left', 'Saída Controlada')
        self.plot_widget.setLabel('bottom', 'Tempo (s)')
        self.plot_widget.setTitle("Resposta do Controle PID")
        self.layout.addWidget(self.plot_widget)

    def _create_spin_box(self, min_val=0.0, max_val=10000.0, decimals=4, default_val=0.0):
        """Cria um QDoubleSpinBox padronizado para entrada/saída de parâmetros PID e SetPoint."""
        sb = QDoubleSpinBox()
        
        # Trata limites grandes (substitui np.inf por um valor prático que o Qt aceita)
        if not np.isfinite(min_val): min_val = -1e6 
        if not np.isfinite(max_val): max_val = 1e6
            
        sb.setRange(min_val, max_val)
        sb.setDecimals(decimals)
        sb.setValue(default_val)
        sb.setAlignment(Qt.AlignCenter)
        return sb

    def _create_line_edit(self, initial_text="---"):
        """Cria um QLineEdit (campo de texto) somente leitura e centralizado para exibição de métricas."""
        le = QLineEdit()
        le.setReadOnly(True)
        le.setAlignment(Qt.AlignCenter)
        le.setText(initial_text)
        return le
    
    def clear_metrics(self):
        """Limpa todos os campos de métricas de desempenho para preparo de uma nova simulação."""
        self.le_tr.setText("--- s")
        self.le_ts.setText("--- s")
        self.le_mp.setText("--- %")
        self.le_ess.setText("---")

    def clear_tuning_fields(self):
        """Limpa e define valores padrão para os campos Kp, Ti, Td, Lambda no modo Manual."""
        self.le_kp.setValue(0.0)
        self.le_ti.setValue(0.01) # Ti é inicializado ligeiramente acima de zero para evitar divisão por zero no Gc(s)
        self.le_td.setValue(0.0)
        self.le_lambda.setValue(1.0)