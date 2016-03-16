library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity csAlu is
	generic(WIDTH : integer);
	port(ctrl : in std_logic_vector(3 downto 0);
		 a : in std_logic_vector(WIDTH - 1 downto 0);
		 b : in std_logic_vector(WIDTH - 1 downto 0);
		 result : out std_logic_vector(WIDTH - 1 downto 0);
		 overflow : out std_logic;
		 zero : out std_logic);
end csAlu;

architecture behavioral of csAlu is

	component alu_1bit is
	generic(carrySelect : boolean := true);
	port(operation : in std_logic_vector(1 downto 0);
		 a : in std_logic;
		 aInvert : in std_logic;
		 b : in std_logic;
		 bInvert : in std_logic;
		 carryIn : in std_logic;
		 less : in std_logic;
		 result : out std_logic;
		 carryOut : out std_logic;
		 set : out std_logic);
	 end component alu_1bit;

	 signal carry_Array : std_logic_vector(WIDTH downto 0) := (others => '0');
	 signal result_array : std_logic_vector(WIDTH - 1 downto 0);
	 signal zero_control : std_logic_vector(WIDTH-1 downto 0) := (others => '0');
	 signal less_Array : std_logic_vector(WIDTH-1 downto 0) := (others => '0');

begin

    -- Carry-Select-ALU-Beschreibung hier einfügen
		carry_Array(0) <= '1' when ctrl(3 downto 2) = "01" else
											'1' when ctrl(3 downto 2) = "10" else
											'0';

		alus:	 for i in 0 to WIDTH - 1 generate

			condition1: if i < WIDTH-1 generate
				gen_alus:		entity work.alu_1bit(structural)
			  		generic map (carrySelect => true)
						port map( operation => ctrl(1 downto 0),
										a => a(i),
										aInvert => ctrl(3),
										b => b(i),
										bInvert => ctrl(2),
										carryIn => carry_Array(i),
										result => result_array(i),
										less => less_Array(i),
										set => open,
										carryOut => carry_Array(i + 1)
										);
			end generate;

			condition2: if i = WIDTH-1 generate
				gen_alus:		entity work.alu_1bit(structural)
						generic map (carrySelect => true)
						port map( operation => ctrl(1 downto 0),
										a => a(i),
										aInvert => ctrl(3),
										b => b(i),
										bInvert => ctrl(2),
										carryIn => carry_Array(i),
										result => result_array(i),
										less => less_Array(i),
										set => less_Array(0),
										carryOut => carry_Array(i + 1)
										);
    	end generate;

		end generate;

		overflow <= '1' when (carry_Array(WIDTH) xor carry_Array(WIDTH - 1)) = '1' else '0';
		result <= result_array;
		zero <= '1' when result_array = zero_control else '0';

end behavioral;
