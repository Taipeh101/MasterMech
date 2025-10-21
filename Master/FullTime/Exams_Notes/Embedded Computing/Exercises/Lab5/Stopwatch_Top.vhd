library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library altera;
use altera.maxplus2.all;

entity Stopwatch is
    Port (
        i_clk      : in  std_logic;                         -- 50 MHz clock
        i_key      : in  std_logic_vector(3 downto 2);      -- KEY[3] = start/stop, KEY[2] = reset
        o_segments : out std_logic_vector(27 downto 0)      -- 4 x 7-segment displays (MSB = leftmost)
    );
end Stopwatch;

architecture Structural of Stopwatch is

    -- Clock & enable
    signal w_clk_100Hz : std_logic;
    signal w_en        : std_logic := '0';
    signal w_clear     : std_logic;

    -- Debounce signals
    signal ff1, ff2    : std_logic := '1';
    signal btn_event   : std_logic;

    -- Counter outputs
    signal q0, q1, q2, q3 : std_logic_vector(3 downto 0);
    signal rco0, rco1, rco2 : std_logic;
begin

    -----------------------------------
    -- Clock Divider Instance (100 Hz)
    -----------------------------------
    u_clk_div : entity work.clk_div
        port map (
            clock_50Mhz  => i_clk,
            clock_1MHz   => open,
            clock_100KHz => open,
            clock_10KHz  => open,
            clock_1KHz   => open,
            clock_100Hz  => w_clk_100Hz,
            clock_10Hz   => open,
            clock_1Hz    => open
        );

    -----------------------------------
    -- Hold button value
    -----------------------------------
    u_jkff : jkff
    port map (
        clk   => not i_key(3),  -- button (active-low) as clock
        j     => '1',
        k     => '1',
        clrn  => '1',           -- no async clear
        prn   => '1',           -- no async preset
        q     => w_en
    );

    -----------------------------------
    -- Reset signal (KEY[2] is active low)
    -----------------------------------
    w_clear <= not i_key(2);
	 

    -----------------------------------
    -- Mod10 Counters
    -----------------------------------

    -- Counter 0 (0.01s)
    u_counter0 : entity work.Mod10Counter
        generic map (N => 4)
        port map (
            i_clk     => w_clk_100Hz,
            i_enable  => w_en,
            i_load    => '0',
            i_clear   => w_clear,
            i_dvector => (others => '0'),
            o_rco     => rco0,
            o_q       => q0
        );

    -- Counter 1 (0.1s)
    u_counter1 : entity work.Mod10Counter
        generic map (N => 4)
        port map (
            i_clk     => w_clk_100Hz,
            i_enable  => w_en and rco0,
            i_load    => '0',
            i_clear   => w_clear,
            i_dvector => (others => '0'),
            o_rco     => rco1,
            o_q       => q1
        );

    -- Counter 2 (1s)
    u_counter2 : entity work.Mod10Counter
        generic map (N => 4)
        port map (
            i_clk     => w_clk_100Hz,
            i_enable  => w_en and rco1,
            i_load    => '0',
            i_clear   => w_clear,
            i_dvector => (others => '0'),
            o_rco     => rco2,
            o_q       => q2
        );

    -- Counter 3 (10s)
    u_counter3 : entity work.Mod10Counter
        generic map (N => 4)
        port map (
            i_clk     => w_clk_100Hz,
            i_enable  => w_en and rco2,
            i_load    => '0',
            i_clear   => w_clear,
            i_dvector => (others => '0'),
            o_rco     => open,
            o_q       => q3
        );

    -----------------------------------
    -- Seven Segment Decoders
    -----------------------------------
    u_seg0 : entity work.sevenSegment port map (i_SW => q0, o_HEX0 => o_segments(6 downto 0));
    u_seg1 : entity work.sevenSegment port map (i_SW => q1, o_HEX0 => o_segments(13 downto 7));
    u_seg2 : entity work.sevenSegment port map (i_SW => q2, o_HEX0 => o_segments(20 downto 14));
    u_seg3 : entity work.sevenSegment port map (i_SW => q3, o_HEX0 => o_segments(27 downto 21));

end Structural;
