library ieee;
use ieee.std_logic_1164.all;
--use ieee.numeric_std.all;

entity TrafficLightFSM is
    port (
        i_clk       : in  std_logic;
        i_reset     : in  std_logic;  -- asynchronous, active-high
        i_emergency : in  std_logic;

        o_NS_R : out std_logic;
        o_NS_Y : out std_logic;
        o_NS_G : out std_logic;
        o_EW_R : out std_logic;
        o_EW_Y : out std_logic;
        o_EW_G : out std_logic
    );
end TrafficLightFSM;

architecture TwoSegment of TrafficLightFSM is

    ---------------------------------------------------------------------
    -- State Declaration
    ---------------------------------------------------------------------
    type t_state is (S0, S1, S2, S3);
    signal r_state_reg, r_state_next : t_state;

begin

    ---------------------------------------------------------------------
    -- Segment 1: State Register
    ---------------------------------------------------------------------
    p_state_register : process(i_clk, i_reset)
    begin
        if i_reset = '1' then
            r_state_reg <= S0;
        elsif (i_clk'event and i_clk = '1') then
            r_state_reg <= r_state_next;
        end if;
    end process;

    ---------------------------------------------------------------------
    -- Segment 2: Next-State and Output Logic
    ---------------------------------------------------------------------
    p_next_state_logic : process(r_state_reg, i_emergency)
    begin
        -- Defaults
        r_state_next <= r_state_reg;
        o_NS_R <= '1'; o_NS_Y <= '1'; o_NS_G <= '1';
        o_EW_R <= '1'; o_EW_Y <= '1'; o_EW_G <= '1';
	
        case r_state_reg is
            when S0 =>
                o_EW_R <= '0';
                if i_emergency = '1' then
		    o_NS_R <= '0';
                else
		    o_NS_G <= '0';
                    r_state_next <= S1;
                end if;

            when S1 =>
                o_EW_R <= '0';
                if i_emergency = '1' then
		    o_NS_R <= '0';
                else
		    o_NS_Y <= '0';
                    r_state_next <= S2;
                end if;

            when S2 =>
                o_NS_R <= '0';
                if i_emergency = '1' then
                    o_EW_R <= '0';
                else
		    o_EW_G <= '0';
                    r_state_next <= S3;
                end if;

            when S3 =>
                o_NS_R <= '0';
                if i_emergency = '1' then
                    o_EW_R <= '0';
                else
		    o_EW_Y <= '0';
                    r_state_next <= S0;
                end if;
        end case;
    end process;

end TwoSegment;

