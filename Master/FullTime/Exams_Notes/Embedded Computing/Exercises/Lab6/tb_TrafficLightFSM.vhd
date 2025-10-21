library ieee;
use ieee.std_logic_1164.all;

entity tb_TrafficLightFSM is
end tb_TrafficLightFSM;

architecture behavior of tb_TrafficLightFSM is

    component TrafficLightFSM
        port (
            i_clk       : in  std_logic;
            i_reset     : in  std_logic;
            i_emergency : in  std_logic;

            o_NS_R      : out std_logic;
            o_NS_Y      : out std_logic;
            o_NS_G      : out std_logic;
            o_EW_R      : out std_logic;
            o_EW_Y      : out std_logic;
            o_EW_G      : out std_logic
        );
    end component;

    -- Testbench signals
    signal i_clk       : std_logic := '0';
    signal i_reset     : std_logic := '0';
    signal i_emergency : std_logic := '0';

    signal o_NS_R, o_NS_Y, o_NS_G : std_logic;
    signal o_EW_R, o_EW_Y, o_EW_G : std_logic;

    constant c_CLK_PERIOD : time := 20 ns;

begin

    -- Instantiate DUT
    uut : TrafficLightFSM
        port map (
            i_clk        => i_clk,
            i_reset      => i_reset,
            i_emergency  => i_emergency,
            o_NS_R       => o_NS_R,
            o_NS_Y       => o_NS_Y,
            o_NS_G       => o_NS_G,
            o_EW_R       => o_EW_R,
            o_EW_Y       => o_EW_Y,
            o_EW_G       => o_EW_G
        );

    -- Clock process
    p_clk : process
    begin
        while true loop
            i_clk <= '0';
            wait for c_CLK_PERIOD / 2;
            i_clk <= '1';
            wait for c_CLK_PERIOD / 2;
        end loop;
    end process;

    -- Stimulus
    process
      begin

        i_reset <= '1';
        wait until (i_clk'event and i_clk = '0');
        i_reset <= '0';
        wait until (i_clk'event and i_clk = '0');

        i_emergency <= '1';
        wait until (i_clk'event and i_clk = '0');
        i_emergency <= '0';
        wait until (i_clk'event and i_clk = '0');

	i_emergency <= '1';
        wait until (i_clk'event and i_clk = '0');
        i_emergency <= '0';
        wait until (i_clk'event and i_clk = '0');

	i_emergency <= '1';
        wait until (i_clk'event and i_clk = '0');
        i_emergency <= '0';
        wait until (i_clk'event and i_clk = '0');
        

        -- Stop simulation
        wait for c_CLK_PERIOD;
        assert false report "Simulation finished." severity failure;
    end process;

end behavior;

