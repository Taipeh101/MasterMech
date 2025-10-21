LIBRARY ieee ;
USE ieee.std_logic_1164.all ;
ENTITY light IS
PORT(x1, x2 : IN STD_LOGIC ;
f : OUT STD_LOGIC);
END light ;
ARCHITECTURE LogicFunction OF light IS
BEGIN
f <= (x1 AND NOT x2) OR (NOT x1 AND x2) ;
END LogicFunction ;
