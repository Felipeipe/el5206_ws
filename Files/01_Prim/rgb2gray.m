function imagen_out = rgb2gray(imagen_in)
    s = size(imagen_in);
    if numel(s)==1
        imagen_out = imagen_in;
    elseif numel(s)==3
        imagen_in = double(imagen_in);
        imagen_out=uint8((imagen_in(:,:,1)*.30)+(imagen_in(:,:,2)*.59)+ (imagen_in(:,:,3)*.11));
    else
        msg = 'error en dimensiones de imagen';
        error(eid,'%s',msg);
    end
end