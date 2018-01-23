--- module (NICHT AENDERN!)
module LascaBot where
--- imports (NICHT AENDERN!)
import Util
import Data.Char
import System.Environment
--- external signatures (NICHT AENDERN!)
getMove   :: String -> String
listMoves :: String -> String

-- *==========================================* --
-- |    HIER BEGINNT EURE IMPLEMENTIERUNG!    | --
-- *==========================================* --

--- types/structures (TODO)
-- Fucking American spelling
data Color = White | Black
data MoveType = Jump | Take | Invalid

    --- ... ---

--- logic (TODO)
getMove   s = concat $ tuplesToFEN $ take 1 $ getLegalMoves s
listMoves s = flattenList $ tuplesToFEN $ getLegalMoves s

flattenList :: [String] -> String
flattenList s =
	if length s > 1 then
		"[" ++ concat (interleave s (interleaveCommas (length s))) ++ "]"
	else
		"[" ++ (concat s) ++ "]"

interleaveCommas :: Int -> [String]
interleaveCommas x = 
	[ "," | x <- [1..(x-1)]]

getLegalMoves :: String -> [(Int,Int)]
getLegalMoves s = filterMoves $ getAllMoves s

filterMoves :: [(Int,Int)] -> [(Int,Int)]
filterMoves x =
	if takesPresent x then
		[ m | m <- x, isTake m ]
	else
		x

tuplesToFEN :: [(Int,Int)] -> [String]
tuplesToFEN x = 
	[ (findField m)++"-"++(findField n) | (m,n) <- x ]

takesPresent :: [(Int,Int)] -> Bool
takesPresent x = 
	let takes = [ m | m <- x, isTake m ]
	in
	if null takes then
		False
	else
		True

-- It's a big function. We expect to deprecate it
getAllMoves :: String -> [(Int,Int)]
getAllMoves s =
	let state = parse s
	in let
	field = getFinal (state!!0)
	player = parseColor (state!!1)
	in
	if length state == 3 then
		getMoveInField field player (findIndex (drop 3 (state!!2)))
	else
		getMovesForPlayer field player

-- Input: board state, player colour and position
-- Output: all possible player moves
getMovesForPlayer :: [String] -> Color -> [(Int,Int)]
getMovesForPlayer f p = 
	let playerFields = getPlayerFields f p
	in concat(map (getMoveInField f p) playerFields)

-- Input: board state, player colour and position
-- Output: possible moves for position
getMoveInField :: [String] -> Color -> Int -> [(Int,Int)]
getMoveInField f p x =
	if (isOfficer [((f!!x)!!0)]) then
		let movesList = map (checkVector f p x) [-8,-6,6,8]
		in [ (x,y) | (x,y) <- movesList, (x,y) /= (-1,-1) ]
	else if p == White then
		let movesList = map (checkVector f p x) [-8,-6]
		in [ (x,y) | (x,y) <- movesList, (x,y) /= (-1,-1) ]
	else
		let movesList = map (checkVector f p x) [8,6]
		in [ (x,y) | (x,y) <- movesList, (x,y) /= (-1,-1) ]

checkVector :: [String] -> Color -> Int -> Int -> (Int,Int)
checkVector f p x v =
	let moveType = checkMoveType f (x,x+v)
	in
	case moveType of
	Jump -> (x,x+v)
	Take -> (x,x+(2*v))
	Invalid -> (-1,-1)

isTake :: (Int,Int) -> Bool
isTake x =
	if (abs (fst x - snd x)) > 10 then
		True
	else
		False

checkMoveType :: [String] -> (Int,Int) -> MoveType
checkMoveType f move
	| isValidTake f move = Take
	| isValidJump f move = Jump
	| otherwise = Invalid

isValidJump :: [String] -> (Int,Int) -> Bool
isValidJump f move =
	let validVector = isValidVector move
	    validJump = ((abs (fst move - snd move)) < 10) && null (f!!(snd move))
	in
	if validVector && validJump then
		True
	else
		False

takeEndSquare :: (Int,Int) -> Int
takeEndSquare m = 
	let end = (snd m) + ((snd m) - (fst m))
	in if end <= 48 && end >= 0 then
		end
	else
		1

jumpToTake :: (Int,Int) -> (Int,Int)
jumpToTake (x,y) = (x, takeEndSquare (x,y))

isValidTake :: [String] -> (Int,Int) -> Bool
isValidTake f move =
	let takeSquare = takeEndSquare move
	    validVector = isValidVector move
	    validVectorExt = isValidVectorExt (jumpToTake move)
	    validTake = ((abs (fst move - takeSquare)) > 10) && null (f!!takeSquare)
	    validMiddle = isValidMiddle f move
	in
	if validVector && validTake && validVectorExt  && validMiddle then
		True
	else
		False

isValidMiddle :: [String] -> (Int,Int) -> Bool
isValidMiddle f move =
	if null $ (f!!(snd move)) then
		False
	else
	let player = parseColor (take 1 (f!!(fst move)))
	    adjcolor = parseColor (take 1 (f!!(snd move)))
	in
	if player == adjcolor then
		False
	else
		True

-- Check the geometrical conditions
-- Input: Board position and movement vector
-- Output: Geometrical validity of move
isValidVector :: (Int,Int) -> Bool
isValidVector (x,v) =
	if (withinBounds (x,v)) && (properVector (x,v)) then
		True
	else
		False

isValidVectorExt :: (Int,Int) -> Bool
isValidVectorExt (x,v) =
	if (withinBounds (x,v)) && (properVectorTake (x,v)) then
		True
	else
		False

withinBounds :: (Int,Int) -> Bool
withinBounds (x,v) = (v >= 0) && (v <= 48) && ((x `mod` 2) == 0)

properVector :: (Int,Int) -> Bool
properVector (x,v) =
	let rowDiff = abs ((div x 7)-(div v 7))
	in if rowDiff == 1 then True else False

properVectorTake :: (Int,Int) -> Bool
properVectorTake (x,v) =
	let rowDiff = abs ((div x 7)-(div v 7))
	in if rowDiff == 2 then True else False

-- Input: board state, player colour
-- Output: list of indices representing player-owned fields
getPlayerFields :: [String] -> Color -> [Int]
getPlayerFields f p = getPlayerFields' f p 0

-- Subfunction containing recursion
-- Input: Same as getPlayerFields and an index
-- Output: list of indices representing player-owned fields
getPlayerFields' :: [String] -> Color -> Int -> [Int]
getPlayerFields' f p 50 = []
getPlayerFields' f p i = (getPlayerFields' f p (i+2)) ++ (if isPlayers (f!!i) p then [i] else [])

-- Input: Stones in a board position and player colour
-- Output: Whether it belongs to the player
isPlayers :: String -> Color -> Bool
isPlayers s p = if (isStone s) then
					if (parseColor [(s!!0)]) == p then
					True
					else False
				else False

    --- ... ---

--- input (TODO)

parse :: String -> [String]
--parse s = parseInput (splitOn " " s)
parse s = splitOn " " s

--parseInput :: [String] -> ...
--parseInput (board:color:[])   = ... (parseColor color) ...
--parseInput (board:color:move:[]) = ... (parseColor color) ...

parseColor :: String -> Color
parseColor "w" = White
parseColor "b" = Black
parseColor "W" = White
parseColor "B" = Black

toInt :: Char -> Int
toInt c = ((ord c) - (ord 'a') + 1)

isOfficer :: String -> Bool
isOfficer "W" = True
isOfficer "B" = True
isOfficer "w" = False
isOfficer "b" = False

isStone :: String -> Bool
isStone s = if not $ null s then True else False

    --- ... ---

--- output (TODO)
colorToString :: Color -> String
colorToString White = "w"
colorToString Black = "b"

instance Show Color where
    show = colorToString
    
instance Eq Color where
    (==) White White = True
    (==) Black Black = True
    (==) _ _ = False    


moveTypeToString :: MoveType -> String
moveTypeToString Jump = "Jump"
moveTypeToString Take = "Take"
moveTypeToString Invalid = "Invalid"

instance Show MoveType where
    show = moveTypeToString
    
instance Eq MoveType where
    (==) Jump Jump = True
    (==) Take Take = True
    (==) Invalid Invalid = True
    (==) _ _ = False    

    --- ... ---
-- !! Additional Functions !! --
interleave :: [String] -> [String] -> [String]
interleave [] ys = ys
interleave (x:xs) ys = x:interleave ys xs

-- Take FEN string
splitOnSlash :: String -> [String]
splitOnSlash x = splitOn "/" x

commaListSmall :: [String]
commaListSmall = [ "," | x <- [1..6]]

commaListBig :: [String]
commaListBig = [ "," | x <- [1..24]]

lascaInit :: String
lascaInit = "b,b,b,b/b,b,b/b,b,b,b/,,/w,w,w,w/w,w,w/w,w,w,w"

lascaInitFull :: String
lascaInitFull = lascaInit++" w"

getFlat :: [String] -> String
getFlat x = concat $ interleave x commaListSmall

getFinal :: String -> [String]
getFinal x = interleave (splitOn "," $ getFlat (splitOnSlash x)) commaListBig 

lascaIndex :: [String]
lascaIndex = ["a7",",","c7",",","e7",",","g7",",","b6",",","d6",",","f6",",","a5",",","c5",",","e5",",","g5",",","b4",",","d4",",","f4",",","a3",",","c3",",","e3",",","g3",",","b2",",","d2",",","f2",",","a1",",","c1",",","e1",",","g1"]

-- findIndex :: [String] -> String -> Int
-- findIndex array elem =

-- TODO: But it works
findPos :: Eq a => a -> [a] -> [Int]
findPos elem = reverse . fst . foldl step ([],0) where
    step (is,i) e = (if e == elem then i:is else is, succ i) 

findIndex' :: Eq a => a -> [a] -> Int
findIndex' elem array = (findPos elem array)!!0

findIndex :: String -> Int
findIndex elem = findIndex' elem lascaIndex

findField :: Int -> String 
findField index = lascaIndex !! index
