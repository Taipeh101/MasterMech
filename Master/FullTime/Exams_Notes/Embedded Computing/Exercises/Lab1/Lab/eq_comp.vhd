-- EQ Component: Checks if A == B
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity eq_comp is
  Port (
   i_A : in STD_LOGIC_VECTOR(1 downto 0);
   i_B : in STD_LOGIC_VECTOR(1 downto 0);
   o_EQ : out STD_LOGIC
  );
end eq_comp;

architecture Behavioral of eq_comp is
SIGNAL status :STD_LOGIC_VECTOR(3 downto 0);

begin
  status <= i_A & i_B;

  process(status)
  begin
    case status is
      when "0000" | "0101" | "1010" | "1111" => o_EQ <= '1';
      when others => o_EQ <= '0';
    end case;
  end process;
end Behavioral;
