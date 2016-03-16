library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity signExtend is
	generic(INPUT_WIDTH : integer;
			OUTPUT_WIDTH : integer);
	port(number : in signed(INPUT_WIDTH - 1 downto 0);
		 signExtNumber : out signed(OUTPUT_WIDTH - 1 downto 0));
end signExtend;

architecture behavioral of signExtend is

begin
	process(number)
	variable zwischen : std_logic_vector(OUTPUT_WIDTH - 1 downto 0);
	variable nr : std_logic_vector(INPUT_WIDTH - 1 downto 0);
	begin
		nr(INPUT_WIDTH - 1 downto 0) := std_logic_vector(number);
		case nr(INPUT_WIDTH - 1) is
			when '0' => zwischen := (others => '0');
			when '1' => zwischen := (others => '1');
			when others => zwischen := (others => 'X');
		end case;
		zwischen(INPUT_WIDTH - 1 downto 0) := std_logic_vector(number);
		signExtNumber <= signed(zwischen);
	end process;
	
end behavioral;
