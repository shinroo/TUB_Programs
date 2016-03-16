library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity clkReduce is
	generic( divisor : integer := 1 );
	port( clk_in  : in  std_logic;
	      clk_out : out std_logic );
end clkReduce;

architecture behavioral of clkReduce is
begin
    process (clk_in)
    variable count : integer := 1; --these are only the values of the variables at the beginning (first process execution)
    variable plusminus : std_logic := '0'; --then they will be probably overwritten
    begin
	if (count > 2*divisor) then --when index out of cycle, go to the beginning
	    count := 1;
	end if;
	if (count <= divisor) then --when in the first 'half' of the signal our clk_out should be '0'
	    plusminus := '0';
	else			   --second 'half' - should be '1'
	    plusminus := '1';
	end if;
	count := count + 1;	   --index++
	
	clk_out <= plusminus;	   --clk_out becomes 0 or 1
    end process;
	

end behavioral;
