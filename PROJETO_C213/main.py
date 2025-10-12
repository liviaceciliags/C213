import sys
from PyQt5.QtWidgets import QApplication
from controllers.main_controller import MainController
from models.pid_model import PIDModel
from views.main_window import MainWindow

def main():
    """
    Função principal. 
    Inicializa o loop de eventos da aplicação (QApplication) e monta a arquitetura MVC.
    """
    # Cria a instância central do QApplication
    app = QApplication(sys.argv)
    
    # 1. Instancia o Modelo (a camada de lógica que contém os algoritmos de identificação e sintonia) [cite: 315, 325, 326]
    model = PIDModel()
    
    # 2. Instancia a View (a camada de apresentação que contém a IHM) [cite: 315, 316]
    view = MainWindow()
    
    # 3. Instancia o Controller (a camada intermediária que conecta eventos da View com a lógica do Model) [cite: 315, 300]
    controller = MainController(model, view)
    
    # Exibe a janela principal
    view.show()
    
    # Inicia o loop de eventos da aplicação
    sys.exit(app.exec_())

if __name__ == '__main__':
    # Garante que a função principal seja chamada apenas quando o script for executado diretamente
    main()