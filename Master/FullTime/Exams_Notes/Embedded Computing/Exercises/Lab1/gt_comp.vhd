
-- GT Component: Checks if A > B
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity gt_comp is
  Port (
    A : in STD_LOGIC_VECTOR(1 downto 0);
    B : in STD_LOGIC_VECTOR(1 downto 0);
    GT : out STD_LOGIC
  );
end gt_comp;

architecture Behavioral of gt_comp is
begin
  process(A, B)
  begin
    if A > B then
      GT <= '1';
    else
      GT <= '0';
    end if;
  end process;
end Behavioral;