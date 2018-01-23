# Systemprogrammierung, Sommersemester 2016

## Übungsblatt 3, Theorie
Gruppe: Hristo Filaretov, Robert Focke, Mikolaj Walukiewicz

### Quellen:
* Kao, Odej. "Systemprogrammierung". 2016. Vorlesungsfolien "3. Scheduling".

### Aufgabe 3.1:
| Prozess | Ausführungszeitpunkt | Laufzeitdauer | Prioritäten |
|:--------|:---------------------|:--------------|:------------|
|    A    |          0           |        6      |      5      |
|    B    |          2           |        1      |      2      |
|    C    |          5           |        7      |      4      |
|    D    |          6           |        8      |      2      |
|    E    |          9           |        3      |      1      |

Gesamte Laufzeitdauer: 25


#### a) Scheduling-Verfahren:

1. SRTN

Shortest Remaining Time Next

|     |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  10 |  11 |  12 |  13 |  14 |  15 |  16 |  17 |  18 |  19 |  20 |  21 |  22 |  23 |  24 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| CPU |  A  |  A  |  B  |  A  |  A  |  A  |  A  |  C  |  C  |  E  |  E  |  E  |  C  |  C  |  C  |  C  |  C  |  D  |  D  |  D  |  D  |  D  |  D  |  D  |  D  |
|  1  |     |     |$A_4$|     |     |  C  |  C  |  D  |  D  |$C_5$|  C  |  C  |  D  |  D  |  D  |  D  |  D  |     |     |     |     |     |     |     |     |
|  2  |     |     |     |     |     |     |  D  |     |     |  D  |  D  |  D  |     |     |     |     |     |     |     |     |     |     |     |     |     |

2. HRRN

Highest Response Ratio Next

Response Ratio $:= \frac{\text{Wartezeit }+\text{ Bedienzeit}}{\text{Bedienzeit}}$

|     |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  10 |  11 |  12 |  13 |  14 |  15 |  16 |  17 |  18 |  19 |  20 |  21 |  22 |  23 |  24 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| CPU |  A  |  A  |  A  |  A  |  A  |  A  |  B  |  C  |  C  |  C  |  C  |  C  |  C  |  C  |  E  |  E  |  E  |  D  |  D  |  D  |  D  |  D  |  D  |  D  |  D  |
|  1  |     |     |  B  |  B  |  B  |  B  |  C  |  D  |  D  |  D  |  D  |  E  |  E  |  E  |  D  |  D  |  D  |     |     |     |     |     |     |     |     |
|  2  |     |     |     |     |     |  C  |  D  |     |     |  E  |  E  |  D  |  D  |  D  |     |     |     |     |     |     |     |     |     |     |     |

3. PRIO-P

Priorities - Preemptive

|     |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  10 |  11 |  12 |  13 |  14 |  15 |  16 |  17 |  18 |  19 |  20 |  21 |  22 |  23 |  24 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| CPU |  A  |  A  |  A  |  A  |  A  |  A  |  C  |  C  |  C  |  C  |  C  |  C  |  C  |  B  |  D  |  D  |  D  |  D  |  D  |  D  |  D  |  D  |  E  |  E  |  E  |
|  1  |     |     |  B  |  B  |  B  |  C  |  B  |  B  |  B  |  B  |  B  |  B  |  B  |  D  |  E  |  E  |  E  |  E  |  E  |  E  |  E  |  E  |     |     |     |
|  2  |     |     |     |     |     |  B  |  D  |  D  |  D  |  D  |  D  |  D  |  D  |  E  |     |     |     |     |     |     |     |     |     |     |     |
|  3  |     |     |     |     |     |     |     |     |     |  E  |  E  |  E  |  E  |     |     |     |     |     |     |     |     |     |     |     |     |


4. Round Robin mit $\tau = 2$

|     |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  10 |  11 |  12 |  13 |  14 |  15 |  16 |  17 |  18 |  19 |  20 |  21 |  22 |  23 |  24 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| CPU |  A  |  A  |  B  |  A  |  A  |  C  |  C  |  A  |  A  |  D  |  D  |  C  |  C  |  E  |  E  |  D  |  D  |  C  |  C  |  E  |  D  |  D  |  C  |  D  |  D  |
|  1  |     |     |$A_4$|     |     |$A_2$|  A  |  D  |  D  |  C  |  C  |  E  |  E  |  D  |  D  |  C  |  C  |  E  |  E  |  D  |  C  |  C  |$D_2$|     |     |
|  2  |     |     |     |     |     |     |  D  |$C_5$|  C  |  E  |  E  |$D_6$|  D  |$C_3$|  C  |$E_1$|  E  |$D_4$|  D  |$C_1$|     |     |     |     |     |

5. Multilevel-Feedback mit $\tau_i = 2^n ( n = 0, 1, ...)$

|        |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  10 |  11 |  12 |  13 |  14 |  15 |  16 |  17 |  18 |  19 |  20 |  21 |  22 |  23 |  24 |
|:------:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| CPU    |  A  |  A  |  A  |  B  |  A  |  A  |  A  |  C  |  D  |  E  |  C  |  C  |  D  |  D  |  E  |  E  |  C  |  C  |  C  |  C  |  D  |  D  |  D  |  D  |  D  |
|$\tau=1$|     |     |  B  |     |     |  C  | C,D |  D  |     |     |     |     |     |     |     |     |     |     |     |     |     |     |     |     |     |
|$\tau=2$|     |     |     |     |     |     |     |     |$C_6$| C,D | D,E | D,E |  E  |  E  |     |     |     |     |     |     |     |     |     |     |     |
|$\tau=4$|     |     |     |$A_3$|     |     |     |     |     |     |     |     |  C  |  C  | C,D | C,D |  D  |  D  |  D  |  D  |     |     |     |     |     |

#### b) Berechnen:

1. SRTN

|Prozess| Antwortzeit | Wartezeit |
|:-----:|:-----------:|:---------:|
|   A   |      7      |     1     |
|   B   |      1      |     0     |
|   C   |     12      |     4     |
|   D   |     19      |    11     |
|   E   |      3      |     0     |

Mittlere Antwortzeit des Systems: $\frac{7+1+12+19+3}{5}=8.4$

Mittlere Wartezeit des Systems: $\frac{1+0+4+11+0}{5}=3.2$

2. HRRN

|Prozess| Antwortzeit | Wartezeit |
|:-----:|:-----------:|:---------:|
|   A   |      6      |     0     |
|   B   |      5      |     4     |
|   C   |      9      |     2     |
|   D   |     19      |    11     |
|   E   |      8      |     5     |

Mittlere Antwortzeit des Systems: $\frac{7+1+12+19+3}{5}=8.2$

Mittlere Wartezeit des Systems: $\frac{0+4+2+11+5}{5}=4.4$

3. PRIO-P

|Prozess| Antwortzeit | Wartezeit |
|:-----:|:-----------:|:---------:|
|   A   |      6      |     0     |
|   B   |     12      |    11     |
|   C   |      8      |     1     |
|   D   |     16      |     8     |
|   E   |     16      |    13     |

Mittlere Antwortzeit des Systems: $\frac{6+12+8+16+16}{5}=11.6$

Mittlere Wartezeit des Systems: $\frac{0+11+1+8+13}{5}=6.6$

4. Round Robin

|Prozess| Antwortzeit | Wartezeit |
|:-----:|:-----------:|:---------:|
|   A   |      9      |     3     |
|   B   |      2      |     1     |
|   C   |     18      |    11     |
|   D   |     19      |    11     |
|   E   |      7      |     4     |

Mittlere Antwortzeit des Systems: $\frac{9+2+18+19+7}{5}=11$

Mittlere Wartezeit des Systems: $\frac{3+1+11+11+4}{5}=6$

5. Multilevel-Feedback 

|Prozess| Antwortzeit | Wartezeit |
|:-----:|:-----------:|:---------:|
|   A   |      7      |     1     |
|   B   |      2      |     1     |
|   C   |     15      |     8     |
|   D   |     19      |    11     |
|   E   |      7      |     4     |

Mittlere Antwortzeit des Systems: $\frac{7+2+15+19+7}{5}=10$

Mittlere Wartezeit des Systems: $\frac{1+1+8+11+4}{5}=5$
