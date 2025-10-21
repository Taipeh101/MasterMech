
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
-- two methods VHDL-87 for EQ VHDL-93 for GT AND LT

--VHDL-87 component is needed
COMPONENT eq_comp
 port (
      i_A       : in  std_logic_vector(1 downto 0);
      i_B       : in  std_logic_vector(1 downto 0);
      o_EQ  : out std_logic
    );
  end component;

begin
 -- EQ Instanz mit VHDL-87
  u_eq: eq_comp
    port map (
      i_A      => i_A,
      i_B      => i_B,
      o_EQ => o_aeq2b
    );

 -- GT Instanz mit VHDL-93
  u_gt: entity work.gt_comp
    port map (
      i_A      => i_a,
      i_B      => i_b,
      o_GT => o_agtb
    );

  -- LT Instanz mit VHDL-93
  u_lt: entity work.lt_comp
    port map (
      i_A      => i_a,
      i_B      => i_b,
      o_lt => o_altb
    );

end Structural;