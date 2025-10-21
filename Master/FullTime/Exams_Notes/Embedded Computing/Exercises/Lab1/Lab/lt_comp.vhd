library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity lt_comp is
    port (
        i_A : in std_logic_vector(1 downto 0);
        i_B : in std_logic_vector(1 downto 0);
        o_lt : out std_logic
    );
end lt_comp;

architecture Behavioral of lt_comp is
Signal w_lt : std_logic_vector(3 downto 0);
begin
w_lt <= i_A & i_B;

        with w_lt select
            o_lt <= '1' when "0011" | "0111" | "1011" | "0110" | "0010" | "0001",   -- Cases where A < B
            '0' when others;         		  -- Cases where A >= B
	
end Behavioral;

