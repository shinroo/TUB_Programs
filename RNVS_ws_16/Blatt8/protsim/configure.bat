echo on
cd messages 
call opp_nmakemake -f -n -I..\common 
copy Makefile.vc Makefile.mak
call nmake -f Makefile.vc
call nmake -f Makefile.vc depend
cd ..
cd support 
call opp_nmakemake -f -n -I..\common 
copy Makefile.vc Makefile.mak
call nmake -f Makefile.vc
call nmake -f Makefile.vc depend
cd ..
cd common 
call opp_nmakemake -f -n -I..\common 
copy Makefile.vc Makefile.mak
call nmake -f Makefile.vc
call nmake -f Makefile.vc depend
cd ..
cd routing 
call opp_nmakemake -f -n -I..\common 
copy Makefile.vc Makefile.mak
call nmake -f Makefile.vc
call nmake -f Makefile.vc depend
cd ..
cd userapps 
call opp_nmakemake -f -n -I..\common 
copy Makefile.vc Makefile.mak
call nmake -f Makefile.vc
call nmake -f Makefile.vc depend
cd ..
cd nodes 
call opp_nmakemake -f -n -I..\common 
copy Makefile.vc Makefile.mak
call nmake -f Makefile.vc
call nmake -f Makefile.vc depend
cd ..
cd networks 
call opp_nmakemake -f -n -I..\common 
copy Makefile.vc Makefile.mak 
call nmake -f Makefile.vc
call nmake -f Makefile.vc depend
cd ..

call opp_nmakemake -f  -u TkEnv -o protsim messages support common routing userapps nodes networks 
copy Makefile.vc Makefile.mak 
call nmake -f Makefile.vc
call nmake -f Makefile.vc depend

