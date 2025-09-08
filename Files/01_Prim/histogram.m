function [H]=histogram(im)

	histo=zeros(1,256);
	for i=1:length(im(:,1))
		for j=1:length(im(1,:))
			histo(im(i,j)+1)=histo(im(i,j)+1)+1;
		end
	end
	x=0:1:255;
	figure; bar(x,histo),xlabel('intensidad'), xlim([0 255]);
	H=histo;


end
