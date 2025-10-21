-- EQ Component: Checks if A == B
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity eq_comp is
  Port (
    A : in STD_LOGIC_VECTOR(1 downto 0);
    B : in STD_LOGIC_VECTOR(1 downto 0);
    EQ : out STD_LOGIC
  );
end eq_comp;

architecture Behavioral of eq_comp is
begin
  process(A, B)
  begin
    case A is
      when "00" => if B = "00" then EQ <= '1'; else EQ <= '0'; end if;
      when "01" => if B = "01" then EQ <= '1'; else EQ <= '0'; end if;
      when "10" => if B = "10" then EQ <= '1'; else EQ <= '0'; end if;
      when "11" => if B = "11" then EQ <= '1'; else EQ <= '0'; end if;
      when others => EQ <= '0';
    end case;
  end process;
end Behavioral;
