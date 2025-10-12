from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel, QLineEdit, QGroupBox, QGridLayout, QHBoxLayout
from PyQt5.QtCore import QLocale, Qt
import pyqtgraph as pg
import numpy as np

class IdentificationTab(QWidget):
    """
    View (Camada V - Visão).
    Cria a interface para carregar dados experimentais, exibir os parâmetros
    do modelo FOPDT (First Order Plus Dead Time) e visualizar o ajuste da curva.
    """
    
    def __init__(self):
        """Constrói o layout e inicializa os widgets da aba de Identificação."""
        super().__init__()
        self.layout = QVBoxLayout(self)
        
        # --- Controles de Carregamento e Status ---
        file_layout = QHBoxLayout()
        self.btn_load = QPushButton("Escolher Arquivo (.mat)") # Botão para carregar o dataset
        file_layout.addWidget(self.btn_load)
        file_layout.addWidget(QLabel("Status:")) 
        self.status_label = QLabel("Nenhuma curva carregada.") # Rótulo que exibe o nome do arquivo
        file_layout.addWidget(self.status_label)
        self.layout.addLayout(file_layout)
        
        # --- Grupo: Parâmetros Identificados (Resultados do FOPDT) ---
        params_group = QGroupBox("Parâmetros do Modelo FOPDT (k, τ, θ)")
        grid_layout = QGridLayout(params_group)
        
        # Campos de saída (Q Line Edit) para k, tau, theta e erro
        self.le_k = self._create_line_edit()      # Ganho estático (k)
        self.le_tau = self._create_line_edit()    # Constante de tempo (tau)
        self.le_theta = self._create_line_edit()  # Atraso de transporte (theta)
        self.le_rmse = self._create_line_edit()   # Erro Quadrático Médio (RMSE/EQM)
        self.le_method = self._create_line_edit() # Método vencedor (Smith ou Sundaresan)

        # Adiciona rótulos e campos ao layout de grade, alinhando os rótulos à direita
        grid_layout.addWidget(QLabel("Ganho (k):"), 0, 0, alignment=Qt.AlignRight)
        grid_layout.addWidget(self.le_k, 0, 1)
        grid_layout.addWidget(QLabel("Const. Tempo (τ):"), 1, 0, alignment=Qt.AlignRight)
        grid_layout.addWidget(self.le_tau, 1, 1)
        grid_layout.addWidget(QLabel("Atraso (θ):"), 2, 0, alignment=Qt.AlignRight)
        grid_layout.addWidget(self.le_theta, 2, 1)
        grid_layout.addWidget(QLabel("EQM (RMSE):"), 3, 0, alignment=Qt.AlignRight)
        grid_layout.addWidget(self.le_rmse, 3, 1)
        grid_layout.addWidget(QLabel("Método Escolhido:"), 4, 0, alignment=Qt.AlignRight)
        grid_layout.addWidget(self.le_method, 4, 1)
        
        self.layout.addWidget(params_group)
        
        # --- Controles de Ação (Botões) ---
        action_layout = QHBoxLayout()
        self.btn_identify = QPushButton("Identificar Modelo FOPDT") # Inicia o cálculo Smith/Sundaresan
        self.btn_identify.setEnabled(False) # Inicia desabilitado, só ativa após o carregamento
        
        self.btn_export_graph = QPushButton("Exportar Gráfico") # Requisito de entrega
        self.btn_export_graph.setEnabled(False) 
        
        action_layout.addWidget(self.btn_identify)
        action_layout.addWidget(self.btn_export_graph)
        self.layout.addLayout(action_layout)
        
        # --- Widget do Gráfico (PyQtGraph) ---
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w') 
        self.plot_widget.setLabel('left', 'Saída Y')
        self.plot_widget.setLabel('bottom', 'Tempo (s)')
        self.plot_widget.setTitle("Curva de Reação (Experimental vs Modelo)")
        self.layout.addWidget(self.plot_widget)

    def _create_line_edit(self):
        """Cria e formata um QLineEdit como campo de saída (somente leitura)."""
        le = QLineEdit()
        le.setReadOnly(True)
        le.setAlignment(Qt.AlignCenter)
        le.setText("---") 
        return le

    def clear_results(self):
        """Reseta todos os campos de resultado de identificação e o gráfico."""
        self.le_k.setText("---")
        self.le_tau.setText("---")
        self.le_theta.setText("---")
        self.le_rmse.setText("---")
        self.le_method.setText("N/A")
        self.plot_widget.clear()
        self.btn_export_graph.setEnabled(False)