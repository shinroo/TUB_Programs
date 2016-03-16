// Aufgabe: Wie schafft man es, sich im System ohne Kenntnis des
//          korrekten Passworts einzuloggen?
//          Was muss im Code modifiziert werden, damit das nicht mehr
//          möglich ist?
//
// Antwort: rot13(Jve frura na qra Fcrvpurenqerffra, qnff mhrefg cnffjbeq
//                vz Fcrvpure vfg, qnaa anzr haq fpuyvrßyvpu pbeerpg_cnffjbeq.
//                Qre Nofgnaq mjvfpura qra Fcrvpurea orgeätg wrjrvyf frpumrua.
//                Jve ortvaara zvg frpumrua oryvrovtra Mrvpura süe qra Anzra.
//                Nyyr sbytraqra Mrvpura jreqra va pbeerpg_cnffjbeq trfpuevrora.)
//          rot13(Jraa jve orv fpnas qvr Sbezngvrehat zvg rvare Znkvznyyäatr
//                irefrura (süasmrua), xöaara jve ireuvaqrea, qnff jve qra
//                ibetrfrurara Fcrvpure üorefpuervgra.)
//
// Bei Fragen gern eine E-Mail an Alexander Elvers oder nachschauen, ob
// ich im Chat bin.

#include <stdio.h>
#include <string.h>


int main() {
	char correct_password[] = "secret";
	char name[16];
	char password[16];
	// printf("%p %p %p\n", correct_password, name, password); // Speicheradressen
	printf("Your name: ");
	if (scanf("%s", name) == 1) {
		printf("Hey %s, please authenticate!\nPassword: ", name);
		if (scanf("%s", password) == 1) {
			if (strcmp(password, correct_password) == 0) {
				printf("You are now logged in.\n");
			} else {
				printf("Password is wrong.\n");
			}
		}
	}

	return 0;
}
