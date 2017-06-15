clc
clear all
close all

%% nacteni dat a parametry chyby
load('import1.mat');

%prefix dat rozhoduje o jejich puvodu
%gt- ground truth
%h- hector slam
%gm- gmapping
%c- cartographer
%priklad: gty je y souradnice polohy robota v ground truth

%koeficienty pro vazeni chyby SLAM implementace
koef_chyby_translace = 1;
koef_chyby_rotace = 1;


%% priprava dat - transformace

%otoceni GT pro pocatecni natoceni = 0
gtx = gtx-gtx(1);
gty = gty-gty(1);
[gtxr, gtyr] = otoc(gtx, gty, gttheta);
gtx = gtxr;
gty = gtyr;

%transformace gttheta, zacatek v 0
uhel = gttheta(1);
for i = 1:length(gttheta)  
    pom = gttheta(i)-uhel;
    if pom < -pi
        gttheta(i) = 2*pi+pom;
    else
        gttheta(i) = pom;
    end
end

%htheta transformace z [0 1] s pocatkem v 1 na [-pi pi] se zacatkem v 0 
for i = 1:length(htheta)
    pom = htheta(i) -0.5;
    if pom < 0
        htheta(i) = 1+pom;
    else
        htheta(i) = pom;
    end
end
htheta = remap(htheta,[0 1],[-pi pi]);

%transformace gmtheta z [-0.5 1] na [-pi pi] se zacatkem v 0
for i = 1:length(gmtheta)
    pom = gmtheta(i) -0.5;
    if pom < 0
        gmtheta(i) = 1+pom;
    else
        gmtheta(i) = pom;
    end
end
gmtheta = remap(gmtheta,[min(gmtheta) max(gmtheta)],[-pi pi]);

%transformace ctheta [7.4e-5 1] na [-pi pi] se zacatkem v 0
for i = 1:length(ctheta)
    pom = ctheta(i) - ((max(ctheta)-min(ctheta))/2);
    if pom < min(ctheta)
        ctheta(i) = 1+pom;
    else
        ctheta(i) = pom;
    end
end
ctheta = remap(ctheta,[min(ctheta) 1],[-pi pi]);

%% GRAFY TRAJEKTORII

figure('position', [200, 0,770, 1080]);%763 pro a4
hold all
subplot(3,1,1)
plot(gtx, gty, 'k-', hx, hy, '-b')
title('Hector SLAM: trajektorie')
legend('ground truth', 'Hector SLAM')
subplot(3,1,2)
plot(gtx, gty, 'k-', gmx, gmy, '-g')
title('gMapping: trajektorie')
legend('ground truth', 'gMapping')
subplot(3,1,3)
plot(gtx, gty, 'k-', cx, cy, '-r')
title('Google Cartographer: trajektorie')
legend('ground truth', 'Hector SLAM')

%% INTERPOLACE GT

% interpolace GT do casu Hector SLAM
gthector = struct ('time',htime(1:41382),'x',zeros(length(htime),1),'y',zeros(length(htime),1),'theta',zeros(length(htime),1));
gthector.x = interp1(gttime, gtx, htime);
gthector.y = interp1(gttime, gty, htime);
gthector.theta = interp1(gttime, gttheta, htime);
%odstraneni NaN hodnot na konci
gthector.x(find(isnan(gthector.x))) = [];
gthector.y(find(isnan(gthector.y))) = [];
gthector.theta(find(isnan(gthector.theta))) = [];

% interpolace GT do casu gMapping
gtgmap = struct ('time',gmtime(1:8332),'x',zeros(length(gmtime),1),'y',zeros(length(gmtime),1),'theta',zeros(length(gmtime),1));
gtgmap.x = interp1(gttime, gtx, gmtime);
gtgmap.y = interp1(gttime, gty, gmtime);
gtgmap.theta = interp1(gttime, gttheta, gmtime);
%odstraneni NaN hodnot na konci
gtgmap.x(find(isnan(gtgmap.x))) = [];
gtgmap.y(find(isnan(gtgmap.y))) = [];
gtgmap.theta(find(isnan(gtgmap.theta))) = [];

% interpolace GT do casu Cartographer
gtcarto = struct ('time',ctime(1:11507),'x',zeros(length(ctime),1),'y',zeros(length(ctime),1),'theta',zeros(length(ctime),1));
gtcarto.x = interp1(gttime, gtx, ctime);
gtcarto.y = interp1(gttime, gty, ctime);
gtcarto.theta = interp1(gttime, gttheta, ctime);
%odstraneni NaN hodnot na konci
gtcarto.x(find(isnan(gtcarto.x))) = [];
gtcarto.y(find(isnan(gtcarto.y))) = [];
gtcarto.theta(find(isnan(gtcarto.theta))) = [];

%% -------------------VYPOCTY CHYB-------------------
%% HECTOR
cash = (htime(1:length(gthector.time))-htime(1))*10^(-6)/60;    %minuty - cas pro grafy
hchyba1 = zeros(length(gthector.time),1);       %celkova chyba urceni polohy
hchybax = zeros(length(gthector.time)-1,1);     %chyba v urceni souradnice polohy x
hchybay = zeros(length(gthector.time)-1,1);     %chyba v urceni souradnice polohy y
hchybatheta = zeros(length(gthector.time)-1,1); %chyba v urceni orientace
hchybac = zeros(length(gthector.time)-1,1);     %chyba implementace
%chyba x
for i = 1:length(gthector.time)-1
    hchybax(i) = abs((hx(i+1)-hx(i))-(gthector.x(i+1)-gthector.x(i)));
end
%chyba y
for i = 1:length(gthector.time)-1
    hchybay(i) = abs((hy(i+1)-hy(i))-(gthector.y(i+1)-gthector.y(i)));
end
%chyba theta
for i = 1:length(gthector.time)-1
    pom1 = abs(abs(htheta(i+1))-abs(htheta(i)));
    pom2 = abs(abs(gthector.theta(i+1))-abs(gthector.theta(i)));
    pom = abs(pom1-pom2);
    hchybatheta(i) = pom;
end
%celkova chyba polohy
for i = 1:length(gthector.time)
    hchyba1(i) = norm(([hx(i) hy(i)])-([gthector.x(i) gthector.y(i)]));
end
%chyba polohy
for i = 1:length(gthector.time)-1
    hchybac(i) = norm(([hx(i+1)-hx(i) hy(i+1)-hy(i)])-([gthector.x(i+1)-gthector.x(i) gthector.y(i+1)-gthector.y(i)]));
end
%chyba dle porovnavani SLAM
hchyba = koef_chyby_translace*hchybac+koef_chyby_rotace*hchybatheta;


%% GMAPPING
casgm = (gmtime(1:length(gtgmap.time))-gmtime(1))*10^(-6)/60;    %minuty - cas pro grafy
gmchybax = zeros(length(gtgmap.time)-1,1);
gmchybay = zeros(length(gtgmap.time)-1,1);
gmchybatheta = zeros(length(gtgmap.time)-1,1);
gmchybac = zeros(length(gtgmap.time)-1,1);
%chyba x
for i = 1:length(gtgmap.time)-1
    gmchybax(i) = abs((gmx(i+1)-gmx(i))-(gtgmap.x(i+1)-gtgmap.x(i)));
end
%chyba y
for i = 1:length(gtgmap.time)-1
    gmchybay(i) = abs((gmy(i+1)-gmy(i))-(gtgmap.y(i+1)-gtgmap.y(i)));
end
%chyba theta
for i = 1:length(gtgmap.time)-1
    %podminka pro osetreni interpolacnich chyb, kdy je uhel mezi casovymi
    %okamziky s -pi a +pi interpolovan jako uhel kolem 0
    if i<length(gtgmap.time)-2 && abs(gtgmap.theta(i))>2.5 && abs(gtgmap.theta(i+1))<0.5 && abs(gtgmap.theta(i+2))>2.5 || i>1 && i<length(gtgmap.time)-1 && abs(gtgmap.theta(i-1))>2.5 && abs(gtgmap.theta(i))<0.5 && abs(gtgmap.theta(i+1))>2.5
        gmchybatheta(i) = gmchybatheta(i-1);
    else
        pom11 = abs(gmtheta(i+1)-gmtheta(i));
        pom1 = min([2*pi-pom11, pom11]);
        pom22 = abs(gtgmap.theta(i+1)-gtgmap.theta(i));
        pom2 = min([2*pi-pom22, pom22]);
        gmchybatheta(i) = abs(pom1-pom2);
    end
end
%celkova chyba polohy
for i = 1:length(gtgmap.time)
    gmchyba1(i) = norm(([gmx(i) gmy(i)])-([gtgmap.x(i) gtgmap.y(i)]));
end
%chyba polohy
for i = 1:length(gtgmap.time)-1
    gmchybac(i) = norm(([gmx(i+1)-gmx(i) gmy(i+1)-gmy(i)])-([gtgmap.x(i+1)-gtgmap.x(i) gtgmap.y(i+1)-gtgmap.y(i)]));
end
%chyba dle porovnavani SLAM
gmchyba = koef_chyby_translace*gmchybac+koef_chyby_rotace*gmchybatheta;


%% CARTOGRAPHER
casc = (ctime(1:length(gtcarto.time))-ctime(1))*10^(-6)/60;    %minuty - cas pro grafy
cchybax = zeros(length(gtcarto.time)-1,1);
cchybay = zeros(length(gtcarto.time)-1,1);
cchybatheta = zeros(length(gtcarto.time)-1,1);
cchybac = zeros(length(gtcarto.time)-1,1);
%chyba x
for i = 1:length(gtcarto.time)-1
    cchybax(i) = abs((cx(i+1)-cx(i))-(gtcarto.x(i+1)-gtcarto.x(i)));
end
%chyba y
for i = 1:length(gtcarto.time)-1
    cchybay(i) = abs((cy(i+1)-cy(i))-(gtcarto.y(i+1)-gtcarto.y(i)));
end
%chyba theta
for i = 1:length(gtcarto.time)-1
    %podminka pro osetreni interpolacnich chyb, kdy je uhel mezi casovymi
    %okamziky s -pi a +pi interpolovan jako uhel kolem 0
    if i<length(gtcarto.time)-2 && abs(gtcarto.theta(i))>2.5 && abs(gtcarto.theta(i+1))<0.5 && abs(gtcarto.theta(i+2))>2.5 || i>1 && i<length(gtcarto.time)-1 && abs(gtcarto.theta(i-1))>2.5 && abs(gtcarto.theta(i))<0.5 && abs(gtcarto.theta(i+1))>2.5
        cchybatheta(i) = cchybatheta(i-1);
    else
        pom11 = abs(ctheta(i+1)-ctheta(i));
        pom1 = min([2*pi-pom11, pom11]);
        pom22 = abs(gtcarto.theta(i+1)-gtcarto.theta(i));
        pom2 = min([2*pi-pom22, pom22]);
        cchybatheta(i) = abs(pom1-pom2);
    end
end
%celkova chyba polohy
for i = 1:length(gtcarto.time)
    cchyba1(i) = norm(([cx(i) cy(i)])-([gtcarto.x(i) gtcarto.y(i)]));
end
%chyba polohy
for i = 1:length(gtcarto.time)-1
    cchybac(i) = norm(([cx(i+1)-cx(i) cy(i+1)-cy(i)])-([gtcarto.x(i+1)-gtcarto.x(i) gtcarto.y(i+1)-gtcarto.y(i)]));
end
%chyba dle porovnavani SLAM
cchyba = koef_chyby_translace*cchybac+koef_chyby_rotace*cchybatheta;

%% ------------------GRAFY------------------

%celkova chyba urceni polohy
maximum = max([max(hchyba1) max(gmchyba1) max(cchyba1)]);
figure('position', [200, 0,770, 1080]);
hold all
subplot(3,1,1)
plot(cash, hchyba1, '-b')
title('Hector SLAM: celkova chyba urceni polohy')
ylim([0 maximum*1.05])
subplot(3,1,2)
plot(casgm, gmchyba1, '-g')
title('gMapping: celkova chyba urceni polohy')
ylim([0 maximum*1.05])
subplot(3,1,3)
plot(casc, cchyba1, '-r')
title('Google Cartographer: celkova chyba urceni polohy')
ylim([0 maximum*1.05])

%inkrementalni chyba urceni polohy
maximum = max([max(hchybac) max(gmchybac) max(cchybac)]);
figure('position', [200, 0,770, 1080]);
hold all
subplot(3,1,1)
plot(cash(2:end), hchybac, '-b')
title('Hector SLAM: inkrementalni chyba urceni polohy')
ylim([0 maximum*1.05])
subplot(3,1,2)
plot(casgm(2:end), gmchybac, '-g')
title('gMapping: inkrementalni chyba urceni polohy')
ylim([0 maximum*1.05])
subplot(3,1,3)
plot(casc(2:end), cchybac, '-r')
title('Google Cartographer: inkrementalni chyba urceni polohy')
ylim([0 maximum*1.05])

%inkrementalni chyba urceni orientace
maximum = max([max(hchybatheta) max(gmchybatheta) max(cchybatheta)]);
figure('position', [200, 0,770, 1080]);
hold all
subplot(3,1,1)
plot(cash(2:end), hchybatheta, '-b')
title('Hector SLAM: inkrementalni chyba urceni orientace')
ylim([0 maximum*1.05])
subplot(3,1,2)
plot(casgm(2:end), gmchybatheta, '-g')
title('gMapping: inkrementalni chyba urceni orientace')
ylim([0 maximum*1.05])
subplot(3,1,3)
plot(casc(2:end), cchybatheta, '-r')
title('Google Cartographer: inkrementalni chyba urceni orientace')
ylim([0 maximum*1.05])

%inkrementalni chyba dle porovnavani SLAM
maximum = max([max(hchyba) max(gmchyba) max(cchyba)]);
figure('position', [200, 0,770, 1080]);
hold all
subplot(3,1,1)
plot(cash(2:end), hchyba, '-b')
title('Hector SLAM: inkrementalni chyba urceni stavu')
ylim([0 maximum*1.05])
subplot(3,1,2)
plot(casgm(2:end), gmchyba, '-g')
title('gMapping: inkrementalni chyba urceni stavu')
ylim([0 maximum*1.05])
subplot(3,1,3)
plot(casc(2:end), cchyba, '-r')
title('Google Cartographer: inkrementalni chyba urceni stavu')
ylim([0 maximum*1.05])

%% DETAILNI GRAFY
%celkova chyba urceni polohy
maximum = max([max(gmchyba1) max(cchyba1)]);
figure('position', [200, 0,770, 720]);
hold all
subplot(2,1,1)
plot(casgm, gmchyba1, '-g')
title('gMapping: celkova chyba urceni polohy')
ylim([0 maximum*1.05])
subplot(2,1,2)
plot(casc, cchyba1, '-r')
title('Google Cartographer: celkova chyba urceni polohy')
ylim([0 maximum*1.05])

%inkrementalni chyba urceni polohy
maximum = max([max(hchybac) max(cchybac)]);
figure('position', [200, 0,770, 720]);
hold all
subplot(2,1,1)
plot(cash(2:end), hchybac, '-b')
title('Hector SLAM: inkrementalni chyba urceni polohy')
ylim([0 maximum*1.05])

subplot(2,1,2)
plot(casc(2:end), cchybac, '-r')
title('Google Cartographer: inkrementalni chyba urceni polohy')
ylim([0 maximum*1.05])

%inkrementalni chyba urceni orientace
maximum = max([max(hchybatheta) max(cchybatheta)]);
figure('position', [200, 0,770, 720]);
hold all
subplot(2,1,1)
plot(cash(2:end), hchybatheta, '-b')
title('Hector SLAM: inkrementalni chyba urceni orientace')
ylim([0 maximum*1.05])
subplot(2,1,2)
plot(casc(2:end), cchybatheta, '-r')
title('Google Cartographer: inkrementalni chyba urceni orientace')
ylim([0 maximum*1.05])

%inkrementalni chyba dle porovnavani SLAM
maximum = max([max(hchyba) max(cchyba)]);
figure('position', [200, 0,770, 720]);
hold all
subplot(2,1,1)
plot(cash(2:end), hchyba, '-b')
title('Hector SLAM: inkrementalni chyba urceni stavu')
ylim([0 maximum*1.05])
subplot(2,1,2)
plot(casc(2:end), cchyba, '-r')
title('Google Cartographer: inkrementalni chyba urceni stavu')
ylim([0 maximum*1.05])


%% 
rmseh = 0;
for i = 1:length(gthector.x)
    rmseh(i) = norm([hx(i) hy(i)] -[gthector.x(i) gthector.y(i)]);
end
rmsegm = 0;
for i = 1:length(gtgmap.x)
    rmsegm(i) = norm([gmx(i) gmy(i)] -[gtgmap.x(i) gtgmap.y(i)]);
end
rmsec = 0;
for i = 1:length(gtcarto.x)
    rmsec(i) = norm([cx(i) cy(i)] -[gtcarto.x(i) gtcarto.y(i)]);
end
%nasledujici chyby jsou ve tvaru [hectorslam, gmapping, carotgrapher]
%RMSE chyba celkove trajektorie
chyba_rmse = [sqrt(mean((rmseh.^2))) sqrt(mean((rmsegm.^2))) sqrt(mean((rmsec.^2)))]

chyby_implementaci_translace = [mean(hchybac) mean(gmchybac) mean(cchybac)] %chyba dle porovnavani slam
chyby_implementaci_rotace = [mean(hchybatheta) mean(gmchybatheta) mean(cchybatheta)] %chyba dle porovnavani slam
chyby_implementaci_celkove = [mean(hchyba) mean(gmchyba) mean(cchyba)] %chyba dle porovnavani slam
smerodatna_odchylka_celkove_chyby = [std(hchyba) std(gmchyba) std(cchyba)] %smerodatna odchylka chyby dle porovnani slam








