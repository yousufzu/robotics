function[R] = myHarrisCorner(Ix, Iy, threshold)

	Ix2 = Ix.^2;
	Iy2 = Iy.^2;
	Ixy = Ix.*Iy;
	Ix2 = myGaussian2(Ix2, 2);
	Iy2 = myGaussian2(Iy2, 2);
	Ixy = myGaussian2(Ixy, 2);
	
	[rows,cols] = size(Ix2);
	R = zeros(rows, cols, 'double');

	for i = 1:rows
		for j = 1:cols
			det = Ix2(i,j) * Iy2(i,j) - Ixy(i,j)^2;
			tra = Ix2(i,j) + Iy2(i,j);
			R(i,j) = det - 0.04*(tra^2);
		end
	end

endfunction


function[B] = myGaussian2(A, sigma)

	% help taken from http://angeljohnsy.blogspot.com/2014/04/gaussian-filter-without-using-matlab.html

	[x,y] = meshgrid(-1:1, -1:1);
	M = size(x,1) - 1;
	N = size(y,1) - 1;
	Exp_comp = -(x.^2+y.^2)/(2*sigma*sigma);
	Kernel= exp(Exp_comp)/(2*pi*sigma*sigma);

	B = myImageFilter(A, Kernel);

endfunction

