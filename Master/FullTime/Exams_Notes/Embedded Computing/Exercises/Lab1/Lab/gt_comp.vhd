
-- GT Component: Checks if A > B
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity gt_comp is
  Port (
    i_A : in STD_LOGIC_VECTOR(1 downto 0);
    i_B : in STD_LOGIC_VECTOR(1 downto 0);
    o_GT : out STD_LOGIC
  );
end gt_comp;

architecture Behavioral of gt_comp is
begin
  process(i_A, i_B)
  begin
    if i_A > i_B then
      o_GT <= '1';
    else
      o_GT <= '0';
    end if;
  end process;
end Behavioral;