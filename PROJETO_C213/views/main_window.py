from PyQt5.QtWidgets import QMainWindow, QTabWidget, QWidget, QVBoxLayout
from .identification_tab import IdentificationTab
from .control_tab import ControlTab

class MainWindow(QMainWindow):
    """
    View (Camada V - Visão).
    Define a janela principal da aplicação, atuando como o container 
    (interface) que hospeda as abas funcionais.
    """
    
    def __init__(self):
        """Inicializa a janela principal e configura a arquitetura de abas."""
        super().__init__()
        
        # [cite_start]Define o título da janela, conforme o nome da disciplina e projeto [cite: 1, 4]
        self.setWindowTitle("Projeto C213 - Identificação e Controle PID")
        # Define o tamanho e a posição inicial da janela
        self.setGeometry(100, 100, 1000, 800)
        
        # --- Configuração do Container de Abas (QTabWidget) ---
        self.tab_widget = QTabWidget()
        self.setCentralWidget(self.tab_widget)
        
        # [cite_start]1. Cria e adiciona a Aba de Identificação de Sistemas [cite: 317]
        self.identification_tab = IdentificationTab()
        self.tab_widget.addTab(self.identification_tab, "Identificação de Sistemas")
        
        # [cite_start]2. Cria e adiciona a Aba de Controle PID [cite: 320]
        self.control_tab = ControlTab()
        self.tab_widget.addTab(self.control_tab, "Controle PID")
        
        # --- Configuração de Fluxo ---
        # Desabilita a aba de Controle PID por padrão, forçando o usuário a realizar
        # a Identificação do sistema primeiro, conforme o fluxo do projeto.
        self.control_tab.setEnabled(False)