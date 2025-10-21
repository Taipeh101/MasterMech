
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity decoder is
    port (
        i_EN : in std_logic;
	i_A  : in std_logic;
	i_B  : in std_logic;	
	
        o_Y  : out std_logic_vector(3 downto 0)
    );
end decoder;

architecture Behavioral of decoder is
Signal w_BA : std_logic_vector(1 downto 0);
begin
w_BA <= i_B & i_A;
process(w_BA, i_EN)
begin
	if i_EN = '1' then o_Y <= "1111";
	else 
		if w_BA = "00" then o_Y <= "1110";
		elsif w_BA = "01" then o_Y <= "1101";
		elsif w_BA = "10" then o_Y <= "1011";
		elsif  w_BA = "11" then o_Y <= "0111";
	end if;
end if;
end process;
end Behavioral;
