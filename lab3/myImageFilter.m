function [img1] = myImageFilter(A, h)

	[x,y] = size(h);
	n = idivide(x, 2, 'floor');

	B = myPad(A, n);
	[rows,cols] = size(B);

	img1 = zeros(rows, cols, 'double');

	for i = (n+1):(rows-n)
		for j = (n+1):(cols-n)
			value = 0;
			%sub = B((i-n):(i+n), (j-n):(j+n));
			%mul = sub.*h;
			%img(i,j) = sum(sum(mul));
			for p = (-1*n):n
				% i + p will go from i-1, i, i+1
				for q = (-1*n):n
					value += B(i+p, j+q) * h(p+n+1, q+n+1);
				end
			end
			img1(i,j) = value;
			value = 0;
		end
	end

endfunction



function[P] = myPad(A, n)

	[rows, cols] = size(A);

	P = zeros(rows + 2*n, cols + 2*n, 'double');

	for i = 1:rows
		for j = 1:cols
			P(i+n, j+n) = A(i,j);
		end
	end

endfunction

