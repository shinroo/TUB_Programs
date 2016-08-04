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
//M = readLTspice("/home/shinroo/Documents/ElNet2016/impedanz_n.txt", "nyquist", 1);
M = readLTspice("/home/shinroo/Documents/ElNet2016/admittanz.txt", "nyquist", 1);
//N = readLTspice("/home/shinroo/Documents/ElNet2016/impedanz_g.txt", "nyquist", 1);
N = readLTspice("/home/shinroo/Documents/ElNet2016/admittanz_g.txt", "nyquist", 1);
Re_1 = M(:,2);
Im_1 = M(:,3);
Re_2 = N(:,2);
Im_2 = N(:,3);
plot(Re_1,Im_1,'r');
el = gce();
el2 = el.children(1);
el2.thickness=3;
plot(Re_2,Im_2,'bo-');
el = gce();
el2 = el.children(1);
el2.thickness=3;
//legend("Impedanzortskurve(sim)", "Impedanzortskurve(gemessen)");
//title("Impedanzortskurvenvergleich");
//xlabel("Re(Z)");
//ylabel("Im(Z)");
legend("Admittanzortskurve(sim)", "Admittanzortskurve(gemessen)");
title("Admittanzortskurvenvergleich");
xlabel("Re(Y)");
ylabel("Im(Y)");
xgrid(1, 1, 3);
