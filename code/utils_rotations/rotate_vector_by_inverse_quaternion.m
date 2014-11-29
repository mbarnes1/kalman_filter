function vout = rotate_vector(vin, q)
%	// return   ( ( q * quaternion(vin) ) * q_conj ) .complex_part

     % Authors: Pieter Abbeel (pabbeel@cs.berkeley.edu)
     %          Adam Coates (acoates@cs.stanford.edu)

vout = quat_multiply( quat_multiply([-q(1:3); q(4)], [vin; 0]), q  );
vout = vout(1:3);

