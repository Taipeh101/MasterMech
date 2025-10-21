library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity Mod10Counter is
    generic (N : integer := 4); -- default 4-bit width
    port (
        i_clk     : in std_logic;
        i_enable  : in std_logic;
        i_load    : in std_logic;
        i_clear   : in std_logic;
        i_dvector : in std_logic_vector(N-1 downto 0);
        o_rco     : out std_logic;
        o_q       : out std_logic_vector(N-1 downto 0)
    );
end Mod10Counter;

architecture TwoSegment of Mod10Counter is
    signal r_reg  : unsigned(N-1 downto 0) := (others => '0'); -- current state
    signal r_next : unsigned(N-1 downto 0);                    -- next state
begin

    process(i_clk, i_clear)
    begin
        if i_clear = '1' then
            r_reg <= (others => '0');
        elsif (i_clk'event and i_clk = '0') then
            r_reg <= r_next;
        end if;
    end process;

    process(i_enable, i_load, i_dvector, r_reg)
    begin
        -- Default: hold current value
        r_next <= r_reg;

        if i_load = '1' then
            if unsigned(i_dvector) > 9 then
                r_next <= (others => '0'); -- load 0 if input > 9
            else
                r_next <= unsigned(i_dvector);
            end if;
        elsif i_enable = '1' then
            if r_reg = 9 then
                r_next <= (others => '0'); -- roll over at 9
            else
                r_next <= r_reg + 1;
            end if;
        end if;
    end process;

    o_q   <= std_logic_vector(r_reg);
    o_rco <= '1' when (r_reg = 9 and i_enable = '1') else '0';

end TwoSegment;
