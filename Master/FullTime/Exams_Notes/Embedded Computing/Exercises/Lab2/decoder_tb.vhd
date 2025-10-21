library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity tb_decoder is
end entity tb_decoder;

architecture behaviour of tb_decoder is

  -- Use Unit Under Test (UUT)
  component decoder
    port (
      i_EN : in  std_logic;
      i_A  : in  std_logic;
      i_B  : in  std_logic;
      o_Y  : out std_logic_vector(3 downto 0)
    );
  end component;

  -- Signals for input and output
  signal test_i_EN : std_logic := '0';
  signal test_i_A  : std_logic := '0';
  signal test_i_B  : std_logic := '0';
  signal test_o_Y  : std_logic_vector(3 downto 0);

begin 

  -- Instantiate the decoder
  uut: decoder
    port map (
      i_EN => test_i_EN,
      i_A  => test_i_A,
      i_B  => test_i_B,
      o_Y  => test_o_Y
    );

  -- Stimulus process
  process
  begin
    -- Test with Enable = 1 (Output should be "1111" regardless of inputs)
    test_i_EN <= '1';
    test_i_A  <= '0';
    test_i_B  <= '0';
    wait for 50 ns;

    test_i_A  <= '1';
    wait for 50 ns;

    test_i_B  <= '1';
    wait for 50 ns;

    test_i_A  <= '0';
    wait for 50 ns;

    -- Test with Enable = 0 (Decoder active)
    test_i_EN <= '0';

    -- Apply all 4 combinations of i_A and i_B
    test_i_A <= '0';
    test_i_B <= '0';
    wait for 50 ns;

    test_i_A <= '0';
    test_i_B <= '1';
    wait for 50 ns;

    test_i_A <= '1';
    test_i_B <= '0';
    wait for 50 ns;

    test_i_A <= '1';
    test_i_B <= '1';
    wait for 50 ns;

    -- End simulation
    assert false 
    report "End of simulation." 
    severity failure;
  end process;
end behaviour;

