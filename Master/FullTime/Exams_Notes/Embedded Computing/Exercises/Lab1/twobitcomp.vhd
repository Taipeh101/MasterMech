
-- Top-Level Component (twobitcomp)
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity twobitcomp is
  Port (
    i_a : in STD_LOGIC_VECTOR(1 downto 0);
    i_b : in STD_LOGIC_VECTOR(1 downto 0);
    o_aeq2b : out STD_LOGIC;
    o_agtb : out STD_LOGIC;
    o_altb : out STD_LOGIC
  );
end twobitcomp;

architecture Structural of twobitcomp is
  component eq_comp
    Port (A, B : in STD_LOGIC_VECTOR(1 downto 0); EQ : out STD_LOGIC);
  end component;
  component gt_comp
    Port (A, B : in STD_LOGIC_VECTOR(1 downto 0); GT : out STD_LOGIC);
  end component;
  component lt_comp
    Port (A, B : in STD_LOGIC_VECTOR(1 downto 0); LT : out STD_LOGIC);
  end component;
begin
  eq_inst: eq_comp port map (i_a, i_b, o_aeq2b);
  gt_inst: gt_comp port map (i_a, i_b, o_agtb);
  lt_inst: lt_comp port map (i_a, i_b, o_altb);
end Structural;