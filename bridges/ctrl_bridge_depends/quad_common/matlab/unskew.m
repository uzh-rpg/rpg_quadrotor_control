function [ v ] = unskew( M )
%UNSKEW Returns the vector v from a skew symetric matrix M
%  0  -w3  w2 
%  w3   0 -w1
% -w2  w1   0
    v = [ M(3,2);M(1,3);M(2,1)  ];
    if ( norm( v+[ M(2,3); M(3,1); M(1,2) ] ) > 0.1 )
       fprintf( 'Matrix not skew symmetric\n' )
        v = []
    end

end

