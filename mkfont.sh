wget https://github.com/levithomason/cmatrix/raw/master/src/mtx.pcf -O mtx.pcf
pcf2bdf -v -o mtx.bdf mtx.pcf 
bdfconv -v -b 2 -f 0 -o mtx.h -n ucg_font_mtx14_mr mtx.bdf
