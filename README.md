# PROJETO C213: Identificação de Sistemas e Controle PID

## 1. Visão Geral do Projeto

Este projeto, desenvolvido para a disciplina **Sistemas Embarcados (C213)**, consiste no desenvolvimento de uma aplicação computacional completa para **modelar, sintonizar e simular um controlador PID** em um processo industrial.

O objetivo é integrar a **teoria de controle clássico** com **habilidades práticas de programação**, aplicando conceitos de identificação de sistemas e sintonia de controladores.

A aplicação utiliza dados experimentais da **curva de reação** para modelar o processo como um sistema de **Primeira Ordem com Atraso de Transporte (FOPDT)**.

---

## 2. Arquitetura da Aplicação (MVC)

A aplicação foi desenvolvida em **Python 3.8 ou superior**, seguindo o padrão de projeto **Modelo-Vista-Controlador (MVC)** para garantir um código bem estruturado e de fácil manutenção.

| Componente | Pasta/Arquivo | Responsabilidade Principal |
| :--- | :--- | :--- |
| **Model (M)** | `models/pid_model.py` | Lógica de cálculo, algoritmos de identificação, regras de sintonia PID, simulação e cálculo de métricas. |
| **View (V)** | `views/*.py` | Criação da Interface Gráfica (IHM) com as abas **Identificação** e **Controle PID**. |
| **Controller (C)** | `controllers/main_controller.py` | Gerenciamento do fluxo de dados e interação entre a View e o Model. |

---

## 3. Configuração do Ambiente e Execução

### 3.1. Dependências

O projeto utiliza a biblioteca oficial de controle para Python (`python-control`), além de ferramentas para interface (`PyQt5`) e cálculo numérico (`NumPy`, `SciPy`).

Instale as dependências executando:

```bash
pip install -r requirements.txt
```

### 3.2. Execução

Execute o arquivo principal a partir do diretório raiz do projeto:

```bash
python main.py
```

---

### Arquivo: `Código_Trabalho_C213.ipynb` presente na pasta PROJETO_C213

Este notebook contém toda a implementação e análise do **Projeto C213 – Identificação de Sistemas e Controle PID**.  
Ele foi desenvolvido em Python e reúne os principais algoritmos utilizados na etapa de modelagem e sintonia do controlador.

Possui leitura dos dados experimentais, identificação do modelo FOPDT, sintonia pelos métodos CHR e ITAE, simulação em malha fechada e comparação dos resultados para avaliar desempenho e estabilidade do sistema.

#### Objetivo do notebook:
Servir como base analítica para o desenvolvimento da aplicação com interface gráfica (PyQt5), permitindo validar e comparar os resultados antes da integração final no software completo.

---

## 4. Funcionalidades de Engenharia Implementadas

### 4.1 Algoritmos Implementados
O sistema foi identificado como FOPDT, cuja função de transferência é:

$$
G(s) = \frac{k \, e^{-\theta s}}{\tau s + 1}
$$

Os métodos usados foram:

• **Smith**→ utiliza os tempos em que a resposta atinge 28,3% e 63,2% da variação final.

• **Sundaresan & Krishnaswamy** → usa 35,3% e 85,3%.

O modelo com menor erro médio quadrático (RMSE) é escolhido como o mais representativo.

---

### 4.2. Aba de Identificação de Sistemas

- **Carregamento de Dados:** Permite carregar arquivos `.mat` com dados experimentais.  
- **Identificação FOPDT:** Implementação dos métodos *Smith* e *Sundaresan & Krishnaswamy*, com cálculo dos parâmetros \( k, 	au, 	heta \).  
- **Seleção de Modelo:** O modelo com menor EQM (RMSE) é selecionado automaticamente.  
- **Visualização:** Exibição da curva de reação (experimental) e do modelo FOPDT sobreposto.  
- **Exportação:** Função para salvar o gráfico como imagem.  

---

### 4.3. Aba de Controle PID

- **Modos de Sintonia:** Suporta Sintonia Automática (método CHR e ITAE) e Sintonia Manual (ajuste fino de \( K_p, T_i, T_d \)).  
- **Métodos de Sintonia (Grupo 3):** Implementação obrigatória do **CHR com Sobressinal** e do **método ITAE**.  
- **Simulação:** Simulação da resposta do sistema em malha fechada (resposta ao degrau).  
- **Métricas de Desempenho:** Cálculo automático de \( t_r \), \( t_s \), \( M_p \) e \( e_{ss} \).  
- **Visualização:** Gráficos interativos com marcadores visuais para *SetPoint* e *Overshoot*.
- **Sintonia Manual:** Permite testar livremente Kp, Ti e Td, observando diretamente os efeitos no comportamento do sistema. 

---

## 5. Análise de Desempenho (Resultados do Grupo 3)

| Métrica | Ziegler-Nichols MA | CHR c/ Sobressinal | ITAE |
| :--- | :--- | :--- | :--- |
| **Kp / Ti / Td** | 6.17 / 28.24 / 7.06 | 4.89 / 58.35 / 6.68 | 4.20 / 32.15 / 4.71 |
| **t<sub>r</sub> (Tempo de Subida)** | 13.000 s (Mais rápido) | 28.500 s | 19.000 s |
| **M<sub>p</sub> (Overshoot)** | 11.82 % | 0.00 % (Mais estável) | 12.48 % |
| **t<sub>s</sub> (Tempo de Acomodação)** | 85.000 s | 87.000 s | 99.000 s |
| **e<sub>ss</sub> (Erro em Regime)** | 0.001 | 0.050 | 0.001 (Excelente) |

---

### Conclusão

A análise comparativa demonstra os **trade-offs entre os métodos de sintonia PID**:

- **CHR com Sobressinal:** Prioriza a estabilidade máxima, resultando em \( M_p = 0.00\% \) — ideal para sistemas que **não toleram ultrapassar o SetPoint**, porém com resposta mais lenta (\( t_r = 28.5s \)).  
- **Ziegler-Nichols MA:** Oferece a resposta **mais rápida** (\( t_r = 13s \)), indicado para sistemas onde a **velocidade é mais importante** que a estabilidade.  
- **ITAE:** Alcança **ótimo erro em regime (0.001)** e tempo intermediário de subida, sendo um **bom compromisso** entre velocidade e precisão.

---

### Limitações e Melhorias Futuras

O projeto cumpre os requisitos propostos, mas o modelo **FOPDT** é uma simplificação e as sintonias não são adaptativas.  
Como melhorias futuras, sugerem-se:

- Implementar **sintonia adaptativa** em tempo real;  
- Adicionar novos métodos de controle, como **IMC**;  
- Integrar o sistema com **hardware real**, permitindo testes práticos e validação experimental.

---

## 6. Autoria

- **Disciplina:** C213 - Sistemas Embarcados  
- **Grupo:** 3  
- **Membros:** Lara Conte Gomes e Lívia Cecília Gomes Silva 
- **Link do Repositório:** [\[INSERIR LINK DO GITHUB\]](https://github.com/liviaceciliags/C213.git)

  
Instituto Nacional de Telecomunicações – Inatel

---
