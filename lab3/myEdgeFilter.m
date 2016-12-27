function[Im Io Ix Iy] = myEdgeFilter(img, sigma)

	B = myGaussian(img, sigma); %blurred

	C = repmat(B,1);

	Iy = myImageFilter(C, [1.0, 2.0, 1.0; 0.0, 0.0, 0.0; -1.0, -2.0, -1.0]); % gradient in y	
	Ix = myImageFilter(B, [-1.0, 0.0, 1.0; -2.0, 0.0, 2.0; -1.0, 0.0, 1.0]); % gradient in x

	[rows,cols] = size(Ix);
	Im = zeros(rows, cols, 'double');
	Io = zeros(rows, cols, 'double');

	for i = 1:rows
		for j = 1:cols
			x = Ix(i,j);
			y = Iy(i,j);
			Im(i,j) = sqrt(x^2 + y^2);
			Io(i,j) = atan2(y, x);
		end
	end

	for i = 2:(rows-1)
		for j = 2:(cols-1)

			degs = (Io(i,j) * 180)/pi;
			cur = Im(i,j);
			comp1 = 0.0;
			comp2 = 0.0;
			if (degs <= 22.5 || degs > 337.5 || (degs > 157.5 && degs <= 202.5))
				comp1 = Im(i+1,j);
				comp2 = Im(i-1,j);
			elseif((degs > 22.5 && degs <= 67.5) || (degs > 202.5 && degs <= 247.5))
				comp1 = Im(i+1,j+1);
				comp2 = Im(i-1,j-1);
			elseif((degs > 67.5 && degs <= 112.5) || (degs > 247.5 && degs <= 292.5))
				comp1 = Im(i, j+1);
				comp2 = Im(i, j-1);
			else
				comp1 = Im(i-1, j+1);
				comp2 = Im(i+1, j-1);
			end

			if(cur < comp1 || cur < comp2)
				Im(i,j) = 0;
			end

		end
	end

endfunction

function[B] = myGaussian(A, sigma)

	% help taken from http://angeljohnsy.blogspot.com/2014/04/gaussian-filter-without-using-matlab.html

	[x,y] = meshgrid(-1:1, -1:1);
	M = size(x,1) - 1;
	N = size(y,1) - 1;
	Exp_comp = -(x.^2+y.^2)/(2*sigma*sigma);
	Kernel= exp(Exp_comp)/(2*pi*sigma*sigma);

	B = myImageFilter(A, Kernel);

endfunction

