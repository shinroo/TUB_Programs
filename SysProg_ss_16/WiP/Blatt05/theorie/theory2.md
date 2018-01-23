## Aufgabe 5.2

* t = 0

Allokation: - 

Belegungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 0 | 0 |
|P2 | 0 | 0 | 0 | 0 |
|P3 | 0 | 0 | 0 | 0 |
|P4 | 0 | 0 | 0 | 0 |

Restanforderungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 4 | 0 | 3 | 3 |
|P2 | 1 | 3 | 0 | 1 |
|P3 | 0 | 2 | 3 | 0 |
|P4 | 0 | 1 | 0 | 3 |

Vorhandene BM:

| A | B | C | D |
|:-:|:-:|:-:|:-:|
| 4 | 4 | 4 | 4 |

* t = 1

Allokation: P1 allocate_r(A,4)

Belegungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 4 | 0 | 0 | 0 |
|P2 | 0 | 0 | 0 | 0 |
|P3 | 0 | 0 | 0 | 0 |
|P4 | 0 | 0 | 0 | 0 |

Restanforderungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 3 | 3 |
|P2 | 1 | 3 | 0 | 1 |
|P3 | 0 | 2 | 3 | 0 |
|P4 | 0 | 1 | 0 | 3 |

Vorhandene BM:

| A | B | C | D |
|:-:|:-:|:-:|:-:|
| 0 | 4 | 4 | 4 |

Mögliche Reihenfolge: P1, P2, P3, P4

* t = 2

Allokation: P1 allocate_r(D,2)

Belegungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 4 | 0 | 0 | 2 |
|P2 | 0 | 0 | 0 | 0 |
|P3 | 0 | 0 | 0 | 0 |
|P4 | 0 | 0 | 0 | 0 |

Restanforderungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 3 | 1 |
|P2 | 1 | 3 | 0 | 1 |
|P3 | 0 | 2 | 3 | 0 |
|P4 | 0 | 1 | 0 | 3 |

Vorhandene BM:

| A | B | C | D |
|:-:|:-:|:-:|:-:|
| 0 | 4 | 4 | 2 |

Mögliche Reihenfolge: P1, P2, P3, P4

* t = 3

Allokation: P2 allocate_r(B,3)

Belegungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 4 | 0 | 0 | 2 |
|P2 | 0 | 3 | 0 | 0 |
|P3 | 0 | 0 | 0 | 0 |
|P4 | 0 | 0 | 0 | 0 |

Restanforderungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 3 | 1 |
|P2 | 1 | 0 | 0 | 1 |
|P3 | 0 | 2 | 3 | 0 |
|P4 | 0 | 1 | 0 | 3 |

Vorhandene BM:

| A | B | C | D |
|:-:|:-:|:-:|:-:|
| 0 | 1 | 4 | 2 |

Mögliche Reihenfolge: P1, P2, P3, P4

* t = 4

Allokation: P2 allocate_r(D,1)

Belegungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 4 | 0 | 0 | 2 |
|P2 | 0 | 3 | 0 | 1 |
|P3 | 0 | 0 | 0 | 0 |
|P4 | 0 | 0 | 0 | 0 |

Restanforderungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 3 | 1 |
|P2 | 1 | 0 | 0 | 0 |
|P3 | 0 | 2 | 3 | 0 |
|P4 | 0 | 1 | 0 | 3 |

Vorhandene BM:

| A | B | C | D |
|:-:|:-:|:-:|:-:|
| 0 | 1 | 4 | 1 |

Mögliche Reihenfolge: P1, P2, P3, P4

* t = 5

Allokation: P3 allocate_r(C,2)

Belegungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 4 | 0 | 0 | 2 |
|P2 | 0 | 3 | 0 | 1 |
|P3 | 0 | 0 | 2 | 0 |
|P4 | 0 | 0 | 0 | 0 |

Restanforderungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 3 | 1 |
|P2 | 1 | 0 | 0 | 0 |
|P3 | 0 | 2 | 1 | 0 |
|P4 | 0 | 1 | 0 | 3 |

Vorhandene BM:

| A | B | C | D |
|:-:|:-:|:-:|:-:|
| 0 | 1 | 2 | 1 |

-----Unischerer Zustand!------

Nach die Allokation P3 $\rightarrow$ (C,2) kann kein weiterer Prozess bis Ende mit dem
vorhandenen BM ausgeführt werden. Das verursacht ein unsichere Zustand. Deswegen wird
Prozess 3 blockiert und von diesem Zeitpunkt ignoriert.

* t = 6

Wird als Teil von P3 ignoriert

* t = 7

Vier A BM-Einheiten werden frei.

* t = 8

Allokation: P1 allocate_r(C,3)

Belegungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 3 | 2 |
|P2 | 0 | 3 | 0 | 1 |
|P3 | - | - | - | - |
|P4 | 0 | 0 | 0 | 0 |

Restanforderungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 0 | 1 |
|P2 | 1 | 0 | 0 | 0 |
|P3 | - | - | - | - |
|P4 | 0 | 1 | 0 | 3 |

Vorhandene BM:

| A | B | C | D |
|:-:|:-:|:-:|:-:|
| 4 | 1 | 1 | 1 |

Mögliche Reihenfolge: P1, P2, P4

* t = 9

Allokation: P4 allocate_r(B,1)

Belegungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 3 | 2 |
|P2 | 0 | 3 | 0 | 1 |
|P3 | - | - | - | - |
|P4 | 0 | 1 | 0 | 0 |

Restanforderungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 0 | 1 |
|P2 | 1 | 0 | 0 | 0 |
|P3 | - | - | - | - |
|P4 | 0 | 0 | 0 | 3 |

Vorhandene BM:

| A | B | C | D |
|:-:|:-:|:-:|:-:|
| 4 | 0 | 1 | 1 |

Mögliche Reihenfolge: P1, P2, P4

* t = 10

Allokation: P4 allocate_r(D,1)

Belegungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 3 | 2 |
|P2 | 0 | 3 | 0 | 1 |
|P3 | - | - | - | - |
|P4 | 0 | 1 | 0 | 1 |

Restanforderungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 0 | 1 |
|P2 | 1 | 0 | 0 | 0 |
|P3 | - | - | - | - |
|P4 | 0 | 0 | 0 | 2 |

Vorhandene BM:

| A | B | C | D |
|:-:|:-:|:-:|:-:|
| 4 | 0 | 1 | 0 |

Mögliche Reihenfolge: P2, P1, P4

* t = 11 & 12

Ignoriert als Teil von P3

* t = 13

Allokation: P2 allocate_r(A,1)

Belegungsmatrix 

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 3 | 2 |
|P2 | 1 | 3 | 0 | 1 |
|P3 | - | - | - | - |
|P4 | 0 | 1 | 0 | 1 |

Restanforderungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 0 | 1 |
|P2 | 0 | 0 | 0 | 0 |
|P3 | - | - | - | - |
|P4 | 0 | 0 | 0 | 2 |

Vorhandene BM:

| A | B | C | D |
|:-:|:-:|:-:|:-:|
| 3 | 0 | 1 | 0 |

Mögliche Reihenfolge: P2, P1, P4

* t = 14

3 B BM-Einheiten werden von P2 freigegeben.

* t = 15

Allokation: P1 allocate_r(D,1)

Belegungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 3 | 3 |
|P2 | 1 | 0 | 0 | 1 |
|P3 | - | - | - | - |
|P4 | 0 | 1 | 0 | 1 |

Restanforderungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | 0 | 0 | 0 | 0 |
|P2 | 0 | 0 | 0 | 0 |
|P3 | - | - | - | - |
|P4 | 0 | 0 | 0 | 2 |

Vorhandene BM:

| A | B | C | D |
|:-:|:-:|:-:|:-:|
| 3 | 0 | 1 | -1 |

---Unmögliche BM Allokation----

Es gibt nicht genug D BM-Einheiten um die P1 Allokation auszuführen.
P1 wird deswegen blockiert und ignoriert. Die vorherallozierte Betriebsmittel werden
nicht freigegeben.

* t = 16

3 C BM-Einheiten werden freigegeben.

* t = 17

Allokation: P4 allocate_r(D,2)

Belegungsmatrix

|   | A | B | C | D |
|:-:|:-:|:-:|:-:|:-:|
|P1 | - | - | 3 | 2 |
|P2 | 1 | 0 | 0 | 1 |
|P3 | - | - | - | - |
|P4 | 0 | 1 | 0 | 3 |

Restanforderungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | - | - | - | 1 |
|P2 | 0 | 0 | 0 | 0 |
|P3 | - | - | - | - |
|P4 | 0 | 0 | 0 | 0 |

Vorhandene BM:

| A | B | C | D |
|:-:|:-:|:-:|:-:|
| 3 | 3 | 1 |-2 |

---Unmögliche BM Allokation----

Es gibt nicht genug D BM-Einheiten um die P4 Allokation auszuführen.
P4 wird deswegen blockiert und ignoriert. Die vorherallozierte Betriebsmittel werden
nicht freigegeben.

* Alle Zeitpunkten bis 22 (inklusiv) werden ignoriert als Teile von ignorierten Prozessen.

* Für t = 23 & 24 werden 1 A und 1 D BM-Einheiten freigegeben.

Die Vorhandene BM-Matrix sieht dann so aus:

Vorhandene BM:

| A | B | C | D |
|:-:|:-:|:-:|:-:|
| 4 | 3 | 1 | 1 |

* t = 25

exit()

Belegungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | - | - | 3 | 2 |
|P2 | 0 | 0 | 0 | 0 |
|P3 | - | - | - | - |
|P4 | - | 1 | - | 1 |

Restanforderungsmatrix

|   | A |B  |C  |D  |
|:-:|:-:|:-:|:-:|:-:|
|P1 | - | - | - | 1 |
|P2 | 0 | 0 | 0 | 0 |
|P3 | - | - | - | - |
|P4 | - | - | - | 2 |

Vorhandene BM:

| A | B | C | D |
|:-:|:-:|:-:|:-:|
| 4 | 3 | 1 | 1 |
