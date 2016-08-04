//---------------------------------------------------------------//
//---------------------------------------------------------------//
//-------------------------- TU Berlin --------------------------//
//--------- Fakultaet IV: Elektrotechnik und Informatik ---------//
//---------------------------------------------------------------//
//--------- Fachgebiet fuer Energieversorgungsnetze und ---------//
//---------- Integration erneuerbarer Energien (SENSE) ----------//
//---------------------------------------------------------------//
//-------------------- Elektrische Netzwerke --------------------//
//-------------------------- Praktikum --------------------------//
//---------------------------------------------------------------//
//---------------------------------------------------------------//
//----- Einlese-Funktion fuer aus LTspice exportierte Daten -----//
//---------------------------------------------------------------//
//-------- Autor: Martin Otto -----------------------------------//
//-------- Version: 1.0 -----------------------------------------//
//-------- Stand: 03.05.2016, 15.15 Uhr -------------------------//
//---------------------------------------------------------------//
//---------------------------------------------------------------//
clc;
clear;
xdel(winsid());
function [M]=readLTspice(fileName, selector, nTraces)
    //Format: Zeitbereich
    if selector=="Time" | selector=="time" then
        tempFileName = part(fileName, 1:$-4) + "_Temp.txt";
        textOriginal = mgetl(fileName);
        [nRows,nColumns] = size(textOriginal);
        textKorrigiert = strsubst(textOriginal(2:nRows), ascii(9), ",");
        mputl(textKorrigiert, tempFileName);
        M = read(tempFileName, -1, 1+nTraces);
        deletefile(tempFileName);
    //Format: Polarform (dB,deg)
    elseif selector=="Bode" | selector=="bode" then
        tempFileName = part(fileName, 1:$-4) + "_Temp.txt";
        textOriginal = mgetl(fileName);
        [nRows,nColumns] = size(textOriginal);
        textKorrigiert = strsubst(textOriginal(2:nRows), "°", "");
        textKorrigiert = strsubst(textKorrigiert, "dB", "");
        textKorrigiert = strsubst(textKorrigiert, "(", "");
        textKorrigiert = strsubst(textKorrigiert, ")", "");
        textKorrigiert = strsubst(textKorrigiert, ascii(9), ",");
        mputl(textKorrigiert, tempFileName);
        M = read(tempFileName, -1, 1+2*nTraces);
        deletefile(tempFileName);
    //Format: kartesische Form (Re,Im)
    elseif selector=="Nyquist" | selector=="nyquist" then
        tempFileName = part(fileName, 1:$-4) + "_Temp.txt";
        textOriginal = mgetl(fileName);
        [nRows,nColumns] = size(textOriginal);
        textKorrigiert = strsubst(textOriginal(2:nRows), ascii(9), ",");
        mputl(textKorrigiert, tempFileName);
        M = read(tempFileName, -1, 1+2*nTraces);
        deletefile(tempFileName);
    //falsche Eingabe
    else
        M = zeros(10,10);
        disp("Falsche Angabe für Format der Datenreihen!");
        disp("--- Art der Datenreihe:");
        disp(selector);
        disp("--- Betroffene Datei:");
        disp(fileName);
    end     
endfunction


M = readLTspice("/home/shinroo/Documents/ElNet2016/Labor_5/lab5datei.txt", "bode",6);

T = M(:,1);

H1 = M(:,2);
H2 = M(:,4);
H3 = M(:,6);

P1 = M(:,3);
P2 = M(:,5);
P3 = M(:,7);

H4 = M(:,8);
H5 = M(:,10);
H6 = M(:,12);

P4 = M(:,8);
P5 = M(:,10);
P6 = M(:,12);

// UNCOMMENT WHOLE SECTION TO SWITCH
//// -------------------------------VERSTARKUNG 100 M Ohm---------------------
//
//plot2d("ln", T, [H1,H2,H3]);
//el = gce();
//el1 = el.children(1);
//el1.thickness=2;
//el1.foreground=2;
//
//el2 = el.children(2);
//el2.thickness=2;
//el2.foreground=3;
//
//el3 = el.children(3);
//el3.thickness=2;
//el3.foreground=4;
//
//title("Verstärkung in dB mit Messwiderstand 100M Ohm");
//legend("Pot auf a = 0", "Pot auf a = 0.5","Pot auf a = 1");
//
//xlabel("f (Hz)");
//ylabel("H (dB)");

// -------------------------------VERSTARKUNG 1 M Ohm-----------

//plot2d("ln", T, [H4,H5,H6]);
//el = gce();
//el1 = el.children(1);
//el1.thickness=2;
//el1.foreground=2;
//
//el2 = el.children(2);
//el2.thickness=2;
//el2.foreground=3;
//
//el3 = el.children(3);
//el3.thickness=2;
//el3.foreground=4;
//
//title("Verstärkung in dB mit Messwiderstand 1M Ohm");
//legend("Pot auf a = 0", "Pot auf a = 0.5","Pot auf a = 1");
//
//xlabel("f (Hz)");
//ylabel("H (dB)");

// -------------------------------Phasenwinkel 100 M Ohm---------------

plot2d("ln", T, [P1,P2,P3]);
el = gce();
el1 = el.children(1);
el1.thickness=2;
el1.foreground=2;

el2 = el.children(2);
el2.thickness=2;
el2.foreground=3;

el3 = el.children(3);
el3.thickness=2;
el3.foreground=4;

title("Phasenwinkel mit Messwiderstand 1M Ohm");
legend("Pot auf a = 0", "Pot auf a = 0.5","Pot auf a = 1");

xlabel("f (Hz)");
ylabel("Phasenwinkel (Grad)");


// -------------------------------Phasenwinkel 100 M Ohm---------------
//
//plot2d("ln", T, [P4,P5,P6]);
//el = gce();
//el1 = el.children(1);
//el1.thickness=2;
//el1.foreground=2;
//
//el2 = el.children(2);
//el2.thickness=2;
//el2.foreground=3;
//
//el3 = el.children(3);
//el3.thickness=2;
//el3.foreground=4;
//
//title("Phasenwinkel mit Messwiderstand 1M Ohm");
//legend("Pot auf a = 0", "Pot auf a = 0.5","Pot auf a = 1");
//
//xlabel("f (Hz)");
//ylabel("Phasenwinkel (Grad)");

xgrid(1, 1, 3);
