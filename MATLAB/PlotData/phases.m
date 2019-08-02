ds = (lf_des_z == 0) & (rf_des_z == 0);


initSS = [];
initDS = [];

isDS = 0;
isSS = 0;

for i = 1:length(ds)
    if(ds(i) == 1)
        if(isDS == 0)
            initDS = [initDS, i];
            isDS = 1;
        end
        isSS = 0;
    end
    if(ds(i) == 0)
        isDS  = 0 ;
        if(isSS == 0)
            initSS = [initSS, i];
            isSS = 1;
        end
    end
    
end
