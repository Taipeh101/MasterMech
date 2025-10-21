library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity lt_comp is
    port (
        a : in std_logic_vector(1 downto 0);
        b : in std_logic_vector(1 downto 0);
        lt : out std_logic
    );
end lt_comp;

architecture Behavioral of lt_comp is
    begin

        with a < b select
            lt <= '1' when true,   -- Cases where A < B
            '0' when others;                   -- Cases where A >= B

end Behavioral;

