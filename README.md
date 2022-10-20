## **SEM5950 - Robôs com Pernas (2022)**
## **José Yecid Moreno Villamizar : 11195127**

---

- ## [Relatorio 2 : Joint Space Motion Control Lab: Control of a 6-DoF Serial Manipulator ](Relatorio%202/README.md)

- ## [Relatorio 3 : Task Space Motion Control](Relatorio%203/README.md)

---

## Mudanças no código

Visando otimizar a velocidade de execução da simulação, modularidade e entendimento os tópicos estudados na aula, criei o dicionário **FLAG** que contem sinais para ativar e desativar algumas regoes do código.

```python
FLAG['SHOW_ANIMATION'] = False
FLAG['SIN_WAVE'] = False
FLAG['SQUARE_WAVE'] = False
FLAG['PD_CONTROL'] = False
FLAG['CRITICAL_DAMPING'] = False
FLAG['GRAVITY_COMPENSATION'] = False
FLAG['FEED_FOWARD'] = False
FLAG['EXTERNAL_FORCE'] = False
FLAG['PD_CONTROL_EA'] = False
FLAG['POSTURAL_TASK'] = False
FLAG['CARTESIAN_ID'] = False
FLAG['CARTESIAN_ID_SIMPLE'] = False
FLAG['LIMS'] = 1
FLAG['POINT'] = '1.7'
```

Aonde:

- *SHOW_ANIMATION*:
    - Oculta a animação do robô, isso permite uma execução mais rápida da simulação
- *SIN_WAVE*:
    - Gera uma trajetória Sinusoidal
- *SQUARE_WAVE*:
    - Gera um degrau 
- *PD_CONTROL*:
    - Controlador PD convencional
- *CRITICAL_DAMPING*:
    - Ativa o cálculo do amortecimento para evitar sobre-sinal, com tempo de resposta rápido
- *GRAVITY_COMPENSATION*:
    - Elimina os efeitos da gravidade sobre o robô
- *FEED_FOWARD*:
    - Calcula o torque Feed-Foward
- *EXTERNAL_FORCE*:
    - Ativa uma força externa constante nalgum atuador
- *PD_CONTROL_EA*:
    - Controlador PD convencional na ponta do atuador
- *POSTURAL_TASK*:
    - Ativa o controle de postura para evitar movimentos indesejados no robô
- *CARTESIAN_ID*:
    - Uso de dinâmica inversa
- *CARTESIAN_ID_SIMPLE*:
    - Uso de dinâmica inversa simplificada
- *LIMS*:
    - São os limites nos eixos *y* dos gráficos, para manter a mesma escala
- *POINT*:
    - É o item do laboratório para analisar

