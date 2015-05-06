function eeState = getEEState( robotState )
%GETEESTATE この関数の概要をここに記述
%   詳細説明をここに記述


[Trans, TREE] = Calculator.getTransMatrix( robotState );
eeState(1) = TREE(1,4);
eeState(2) = TREE(2,4);
eeState(3) = TREE(3,4);
eeState(4) = atan2(TREE(3,2), TREE(3,3) );
eeState(5) = atan2( -TREE(3,1), sqrt(TREE(3,2)^2 + TREE(3,3)^2) );
eeState(6) = atan2( TREE(2,1), TREE(1,1) );


end

