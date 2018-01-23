# SysProg SS 16
## Hausaufgabenblatt 1, Theorie

## Hristo Filaretov, Robert Focke, Mikolaj Walukiewicz

### Quellenangabe:
Kao, Odej. "Systemprogrammierung". 2016. Vorlesungsfolien "1. Rechnerarchitektur und Betriebssysteme".

Jäger, Michael. Betriebssyteme. 1st ed. Technische Hochschule Mittelhessen, 2016. Web. 6 May 2016.

"Speicherdirektzugriff". de.wikipedia.org. N.p., 2016. Web. 6 May 2016.

#### Aufgabe 1.1

a) Modi des Betriebssystems (Kao, 20)

Benutzermodus

* Einige Befehle/Register nicht zugreifbar
* wird von Benutzerprogrammen benutzt

Systemmodus

* Alle Befehle/Register können benutzt werden
* wird von dem Betriebssystem benutzt

b) Kalter und heißer Cache (Kao, 10)

Man nennt Cache "kalt" falls die Cacheinhalte nicht den vom Programm referenzierten Daten entsprechen. Cache ist normalerweise kalt wenn ein neues Programm geladen wird. 
Cache wird als "heiß" bezeichnet, falls seine Inhalte die Daten des aktuellen Programms entsprechen, dies ist normalerweise der Fall nachdem Vorlaufzeit eines Programms.
Heißer Cache hat eine höhere Trefferwahrscheinlichkeit, welche eine höhere Effizienz verursacht (da die notwendige Daten schon im Cache sind, und nicht im Hauptspeicher nochmal
gesucht werden müssen).


#### Aufgabe 1.2
* a) Interrupts (Kao, 26-32)

Interrupts sind spezielle Signale, die die CPU über bestimmte Ereignisse im System informieren (wie z.B. das Ende einer Datenübertragung).

* b) DMA vs. speicherbasierte E/A (Kao, 23-25)

"Direct memory access" ist schneller als speicherbasierte E/A, das Gerät kann selbst direkt den Speicher überschreiben, ohne das Prozessor zu benutzen.

* c) DMA spezifische Komponente (Jäger 146)

Um DMA zu ermöglichen wird einen DMA-Controller benötigt. Der DMA-Controller ist ein Teil der Steuereinheit eines Geräts, das nur Datatransfer ausführt. Der DMA-Controller braucht also Zugang zum Hauptspeicher, also einen Bereich von physikalischen Adressen im Speicher (von kernel zugeteilt). Jedes Gerät hat einen separaten Adressenbereich im RAM, sodass es nicht auf den Speicherbereich von den anderen Geräten zugreift.

* d) Schrittweise Beschreibung vom Ablauf eines DMA beim Lesen von Daten ("Speicherdirektzugriff")

    * 0.) Kernel hat am Anfang (bei der Initialisierung) Platz im Hauptspeicher für das Gerät reserviert und hat er das Gerät darüber informiert.
	* 1.) E/A Gerät möchte etwas übertragen
	* 2.) DMA-Controller trennt CPU vom Bussystem - besser: benutzt den Bus (shares, nutzt halt die gemeinsame Schiene parallel mit dem CPU)
	* 3.) DMA-Controller führt Anforderung aus (liefert die Daten aus der Festplatte in den Hauptspeicher)
	* 4.) Verbindung zwischen CPU und Bussystem wieder hergestellt (oder dem CPU wird nicht mehr "gestört")
