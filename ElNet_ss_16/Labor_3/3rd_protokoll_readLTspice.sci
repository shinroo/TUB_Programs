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


D = readLTspice("/home/shinroo/Documents/ElNet2016/Labor_4/sim.txt", "time",6);

T = D(:,1);
U = D(:,2);
I1L = D(:,5);
I2L = D(:,4);
I1R = D(:,6);
I2R = D(:,3);

plot2d(T, I1R);
el = gce();
el1 = el.children(1);
el1.thickness=2;
el1.foreground=2;

plot2d(T, I2R);
el = gce();
el1 = el.children(1);
el1.thickness=2;
el1.foreground=5;

title("Ströme des rechteren Teils");
legend("I1R","I2R");

xlabel("t (s)");
ylabel("I (A)");



xgrid(1, 1, 3);
