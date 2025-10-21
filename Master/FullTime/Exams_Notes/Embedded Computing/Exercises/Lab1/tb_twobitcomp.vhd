library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all; 

entity tb_twobitcomp is
end entity tb_twobitcomp;

architecture behaviour of tb_twobitcomp is

  -- use UNIT UNDER TEST (UUT)
-- has to match my entity from twobitcomp
  component twobitcomp
    port (
      i_A       : in  std_logic_vector(1 downto 0);
      i_B       : in  std_logic_vector(1 downto 0);
      o_aeq2b   : out std_logic;
      o_agtb    : out std_logic;
      o_altb    : out std_logic
    );
  end component;

signal test_inA       : std_logic_vector(1 downto 0) := "00";
  signal test_inB       : std_logic_vector(1 downto 0) := "00";
  signal test_out_aeq2b : std_logic; -- dont assign a value to an output
  signal test_out_agtb  : std_logic;
  signal test_out_altb  : std_logic;


begin 

  uut: twobitcomp
    port map (
      i_A       => test_inA,
      i_B       => test_inB,
      o_aeq2b   => test_out_aeq2b,
      o_agtb    => test_out_agtb,
      o_altb    => test_out_altb
    );

 
 
  process
  begin
    -- Apply all 16 combinations of i_a and i_b

test_inA <= "00";
test_inB <= "00";
wait for 50 ns;

test_inA <= "00";
test_inB <= "01";
wait for 50 ns;

test_inA <= "00";
test_inB <= "10";
wait for 50 ns;

test_inA <= "00";
test_inB <= "11";
wait for 50 ns;

test_inA <= "01";
test_inB <= "00";
wait for 50 ns;

test_inA <= "01";
test_inB <= "01";
wait for 50 ns;

test_inA <= "01";
test_inB <= "10";
wait for 50 ns;

test_inA <= "01";
test_inB <= "11";
wait for 50 ns;

test_inA <= "10";
test_inB <= "00";
wait for 50 ns;

test_inA <= "10";
test_inB <= "01";
wait for 200 ns;

test_inA <= "10";
test_inB <= "10";
wait for 50 ns;

test_inA <= "10";
test_inB <= "11";
wait for 50 ns;

test_inA <= "11";
test_inB <= "00";
wait for 50 ns;

test_inA <= "11";
test_inB <= "01";
wait for 50 ns;

test_inA <= "11";
test_inB <= "10";
wait for 50 ns;

test_inA <= "11";
test_inB <= "11";
wait for 50 ns;



assert false 
report "End of simulation." 
severity failure;
  end process;
end behaviour;