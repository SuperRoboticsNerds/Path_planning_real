% Automatically generated by solvePlanningProblem using
% #nodes=100
% #edges per node10
% step size for collision check 0.05
% xMin=0
% xMax=10
% yMin=0
% yMax=10
% worldModel="test"
% problem spec file "problem2.txt"

clf
hold on
plot([0.5 1.5],[1 1]);
plot([1.5 1.5],[1 10]);
plot([1.5 0.5],[10 10]);
plot([0.5 0.5],[10 1]);
plot([2.5 3.5],[0 0]);
plot([3.5 3.5],[0 9]);
plot([3.5 2.5],[9 9]);
plot([2.5 2.5],[9 0]);
plot([4.5 5.5],[1 1]);
plot([5.5 5.5],[1 10]);
plot([5.5 4.5],[10 10]);
plot([4.5 4.5],[10 1]);
plot([6.5 7.5],[0 0]);
plot([7.5 7.5],[0 9]);
plot([7.5 6.5],[9 9]);
plot([6.5 6.5],[9 0]);
plot([0 0],[0 0]);
plot([0 0],[0 0]);
plot([0 0],[0 0]);
plot([0 0],[0 0]);
plot(5.8601,1.49491,'.g')
plot(1.70265,7.221,'.g')
plot(1.65688,9.29783,'.g')
plot(5.82931,4.63547,'.g')
plot(7.98199,8.3534,'.g')
plot(4.36933,0.675732,'.g')
plot(2.00757,3.61369,'.g')
plot(5.83068,8.13016,'.g')
plot(8.83602,5.31897,'.g')
plot(6.07014,2.95894,'.g')
plot(4.0214,9.16122,'.g')
plot(4.32925,4.97177,'.g')
plot(1.6275,6.69239,'.g')
plot(5.99022,1.67586,'.g')
plot(2.252,1.81952,'.g')
plot(6.31133,0.233989,'.g')
plot(0.172923,0.680657,'.g')
plot(4.29435,2.3476,'.g')
plot(7.51458,0.125023,'.g')
plot(0.477754,6.3506,'.g')
plot(9.30954,8.74511,'.g')
plot(9.38223,3.33094,'.g')
plot(7.90634,3.71148,'.g')
plot(8.30271,2.89608,'.g')
plot(3.41278,9.93021,'.g')
plot(9.58847,0.335065,'.g')
plot(2.01093,5.4666,'.g')
plot(5.70059,7.57113,'.g')
plot(9.00291,6.61031,'.g')
plot(9.75162,3.29726,'.g')
plot(8.95791,7.2662,'.g')
plot(3.42228,9.43566,'.g')
plot(3.61681,8.86627,'.g')
plot(5.98355,2.92635,'.g')
plot(7.61138,5.36579,'.g')
plot(6.25729,5.51772,'.g')
plot(9.07727,4.56,'.g')
plot(8.4138,2.49004,'.g')
plot(4.49021,8.00227,'.g')
plot(3.58095,4.83603,'.g')
plot(8.55028,2.1612,'.g')
plot(5.56811,8.88074,'.g')
plot(3.87588,9.18491,'.g')
plot(7.74701,9.85944,'.g')
plot(2.11126,5.35839,'.g')
plot(2.92855,9.28991,'.g')
plot(7.29218,9.61765,'.g')
plot(5.12358,0.873126,'.g')
plot(4.45368,8.29499,'.g')
plot(1.85228,7.61197,'.g')
plot(9.77317,2.64931,'.g')
plot(8.70446,5.23163,'.g')
plot(4.11237,0.965419,'.g')
plot(3.93729,8.09388,'.g')
plot(0.352578,6.86584,'.g')
plot(4.28461,4.67598,'.g')
plot(6.76277,9.40819,'.g')
plot(5.5491,1.21645,'.g')
plot(7.70318,7.40138,'.g')
plot(8.82842,4.87018,'.g')
plot(7.80393,8.6016,'.g')
plot(7.51949,6.50839,'.g')
plot(3.83323,4.60902,'.g')
plot(5.57444,4.23843,'.g')
plot(9.80497,6.3993,'.g')
plot(9.80717,7.02274,'.g')
plot(2.44938,3.74445,'.g')
plot(9.94708,4.8949,'.g')
plot(4.30309,2.7255,'.g')
plot(7.92629,2.00627,'.g')
plot(0.12688,6.75472,'.g')
plot(4.43921,9.18954,'.g')
plot(9.00496,5.22016,'.g')
plot(9.4586,6.94011,'.g')
plot(3.96286,3.42808,'.g')
plot(3.01022,9.07948,'.g')
plot(6.23004,3.62051,'.g')
plot(1.5799,6.17711,'.g')
plot(8.51541,8.7563,'.g')
plot(1.4818,0.813248,'.g')
plot(7.56796,1.70122,'.g')
plot(9.5395,2.92428,'.g')
plot(6.09715,3.9787,'.g')
plot(2.11382,5.10211,'.g')
plot(9.19887,9.24896,'.g')
plot(9.68151,8.65746,'.g')
plot(6.18908,0.660208,'.g')
plot(7.92323,0.151931,'.g')
plot(4.08829,0.933444,'.g')
plot(9.23141,0.318324,'.g')
plot(4.55396,0.811316,'.g')
plot(6.49544,3.06937,'.g')
plot(9.56762,9.38239,'.g')
plot(5.88787,1.04942,'.g')
plot(0.19564,0.712646,'.g')
plot(2.41386,2.1976,'.g')
plot(6.17631,2.8017,'.g')
plot(3.61313,5.37517,'.g')
plot(2.05066,3.29464,'.g')
plot(4.03264,8.23974,'.g')
if 0
  plot([1.65688,1.70265] ,[9.29783, 7.221], 'k')
  plot([5.82931,5.8601] ,[4.63547, 1.49491], 'k')
  plot([2.00757,1.70265] ,[3.61369, 7.221], 'k')
  plot([2.00757,1.65688] ,[3.61369, 9.29783], 'k')
  plot([5.83068,5.82931] ,[8.13016, 4.63547], 'k')
  plot([5.83068,5.8601] ,[8.13016, 1.49491], 'k')
  plot([8.83602,7.98199] ,[5.31897, 8.3534], 'k')
  plot([6.07014,5.8601] ,[2.95894, 1.49491], 'k')
  plot([6.07014,5.82931] ,[2.95894, 4.63547], 'k')
  plot([6.07014,5.83068] ,[2.95894, 8.13016], 'k')
  plot([4.0214,1.65688] ,[9.16122, 9.29783], 'k')
  plot([4.0214,4.36933] ,[9.16122, 0.675732], 'k')
  plot([4.32925,4.0214] ,[4.97177, 9.16122], 'k')
  plot([4.32925,4.36933] ,[4.97177, 0.675732], 'k')
  plot([1.6275,1.70265] ,[6.69239, 7.221], 'k')
  plot([1.6275,1.65688] ,[6.69239, 9.29783], 'k')
  plot([1.6275,2.00757] ,[6.69239, 3.61369], 'k')
  plot([5.99022,5.8601] ,[1.67586, 1.49491], 'k')
  plot([5.99022,6.07014] ,[1.67586, 2.95894], 'k')
  plot([5.99022,5.82931] ,[1.67586, 4.63547], 'k')
  plot([5.99022,5.83068] ,[1.67586, 8.13016], 'k')
  plot([2.252,2.00757] ,[1.81952, 3.61369], 'k')
  plot([2.252,1.6275] ,[1.81952, 6.69239], 'k')
  plot([2.252,1.70265] ,[1.81952, 7.221], 'k')
  plot([6.31133,5.8601] ,[0.233989, 1.49491], 'k')
  plot([6.31133,5.99022] ,[0.233989, 1.67586], 'k')
  plot([6.31133,4.36933] ,[0.233989, 0.675732], 'k')
  plot([6.31133,6.07014] ,[0.233989, 2.95894], 'k')
  plot([6.31133,5.82931] ,[0.233989, 4.63547], 'k')
  plot([6.31133,5.83068] ,[0.233989, 8.13016], 'k')
  plot([4.29435,4.36933] ,[2.3476, 0.675732], 'k')
  plot([4.29435,4.32925] ,[2.3476, 4.97177], 'k')
  plot([7.51458,8.83602] ,[0.125023, 5.31897], 'k')
  plot([9.30954,7.98199] ,[8.74511, 8.3534], 'k')
  plot([9.30954,8.83602] ,[8.74511, 5.31897], 'k')
  plot([9.38223,8.83602] ,[3.33094, 5.31897], 'k')
  plot([9.38223,7.98199] ,[3.33094, 8.3534], 'k')
  plot([7.90634,9.38223] ,[3.71148, 3.33094], 'k')
  plot([7.90634,8.83602] ,[3.71148, 5.31897], 'k')
  plot([7.90634,7.51458] ,[3.71148, 0.125023], 'k')
  plot([8.30271,7.90634] ,[2.89608, 3.71148], 'k')
  plot([8.30271,9.38223] ,[2.89608, 3.33094], 'k')
  plot([8.30271,8.83602] ,[2.89608, 5.31897], 'k')
  plot([8.30271,7.51458] ,[2.89608, 0.125023], 'k')
  plot([3.41278,4.0214] ,[9.93021, 9.16122], 'k')
  plot([3.41278,1.65688] ,[9.93021, 9.29783], 'k')
  plot([3.41278,4.32925] ,[9.93021, 4.97177], 'k')
  plot([9.58847,8.30271] ,[0.335065, 2.89608], 'k')
  plot([9.58847,9.38223] ,[0.335065, 3.33094], 'k')
  plot([9.58847,7.90634] ,[0.335065, 3.71148], 'k')
  plot([9.58847,8.83602] ,[0.335065, 5.31897], 'k')
  plot([2.01093,1.6275] ,[5.4666, 6.69239], 'k')
  plot([2.01093,1.70265] ,[5.4666, 7.221], 'k')
  plot([2.01093,2.00757] ,[5.4666, 3.61369], 'k')
  plot([2.01093,2.252] ,[5.4666, 1.81952], 'k')
  plot([2.01093,1.65688] ,[5.4666, 9.29783], 'k')
  plot([5.70059,5.83068] ,[7.57113, 8.13016], 'k')
  plot([5.70059,5.82931] ,[7.57113, 4.63547], 'k')
  plot([9.00291,8.83602] ,[6.61031, 5.31897], 'k')
  plot([9.00291,7.98199] ,[6.61031, 8.3534], 'k')
  plot([9.00291,9.30954] ,[6.61031, 8.74511], 'k')
  plot([9.00291,7.90634] ,[6.61031, 3.71148], 'k')
  plot([9.00291,9.38223] ,[6.61031, 3.33094], 'k')
  plot([9.00291,8.30271] ,[6.61031, 2.89608], 'k')
  plot([9.75162,9.38223] ,[3.29726, 3.33094], 'k')
  plot([9.75162,8.30271] ,[3.29726, 2.89608], 'k')
  plot([9.75162,7.90634] ,[3.29726, 3.71148], 'k')
  plot([9.75162,8.83602] ,[3.29726, 5.31897], 'k')
  plot([9.75162,9.58847] ,[3.29726, 0.335065], 'k')
  plot([9.75162,9.00291] ,[3.29726, 6.61031], 'k')
  plot([8.95791,9.00291] ,[7.2662, 6.61031], 'k')
  plot([8.95791,7.98199] ,[7.2662, 8.3534], 'k')
  plot([8.95791,9.30954] ,[7.2662, 8.74511], 'k')
  plot([8.95791,8.83602] ,[7.2662, 5.31897], 'k')
  plot([8.95791,7.90634] ,[7.2662, 3.71148], 'k')
  plot([8.95791,9.38223] ,[7.2662, 3.33094], 'k')
  plot([8.95791,9.75162] ,[7.2662, 3.29726], 'k')
  plot([3.42228,3.41278] ,[9.43566, 9.93021], 'k')
  plot([3.42228,4.0214] ,[9.43566, 9.16122], 'k')
  plot([3.42228,1.65688] ,[9.43566, 9.29783], 'k')
  plot([3.42228,4.32925] ,[9.43566, 4.97177], 'k')
  plot([3.61681,4.0214] ,[8.86627, 9.16122], 'k')
  plot([3.61681,3.42228] ,[8.86627, 9.43566], 'k')
  plot([3.61681,3.41278] ,[8.86627, 9.93021], 'k')
  plot([3.61681,4.32925] ,[8.86627, 4.97177], 'k')
  plot([5.98355,6.07014] ,[2.92635, 2.95894], 'k')
  plot([5.98355,5.99022] ,[2.92635, 1.67586], 'k')
  plot([5.98355,5.8601] ,[2.92635, 1.49491], 'k')
  plot([5.98355,5.82931] ,[2.92635, 4.63547], 'k')
  plot([5.98355,6.31133] ,[2.92635, 0.233989], 'k')
  plot([7.61138,8.83602] ,[5.36579, 5.31897], 'k')
  plot([7.61138,7.90634] ,[5.36579, 3.71148], 'k')
  plot([7.61138,9.00291] ,[5.36579, 6.61031], 'k')
  plot([7.61138,8.95791] ,[5.36579, 7.2662], 'k')
  plot([7.61138,8.30271] ,[5.36579, 2.89608], 'k')
  plot([7.61138,9.38223] ,[5.36579, 3.33094], 'k')
  plot([6.25729,5.82931] ,[5.51772, 4.63547], 'k')
  plot([6.25729,5.70059] ,[5.51772, 7.57113], 'k')
  plot([6.25729,6.07014] ,[5.51772, 2.95894], 'k')
  plot([6.25729,5.98355] ,[5.51772, 2.92635], 'k')
  plot([6.25729,5.83068] ,[5.51772, 8.13016], 'k')
  plot([9.07727,8.83602] ,[4.56, 5.31897], 'k')
  plot([9.07727,9.38223] ,[4.56, 3.33094], 'k')
  plot([9.07727,9.75162] ,[4.56, 3.29726], 'k')
  plot([9.07727,7.90634] ,[4.56, 3.71148], 'k')
  plot([9.07727,7.61138] ,[4.56, 5.36579], 'k')
  plot([9.07727,8.30271] ,[4.56, 2.89608], 'k')
  plot([9.07727,9.00291] ,[4.56, 6.61031], 'k')
  plot([9.07727,8.95791] ,[4.56, 7.2662], 'k')
  plot([8.4138,8.30271] ,[2.49004, 2.89608], 'k')
  plot([8.4138,9.38223] ,[2.49004, 3.33094], 'k')
  plot([8.4138,7.90634] ,[2.49004, 3.71148], 'k')
  plot([8.4138,9.75162] ,[2.49004, 3.29726], 'k')
  plot([8.4138,9.07727] ,[2.49004, 4.56], 'k')
  plot([8.4138,9.58847] ,[2.49004, 0.335065], 'k')
  plot([4.49021,3.61681] ,[8.00227, 8.86627], 'k')
  plot([4.49021,4.0214] ,[8.00227, 9.16122], 'k')
  plot([4.49021,3.42228] ,[8.00227, 9.43566], 'k')
  plot([4.49021,3.41278] ,[8.00227, 9.93021], 'k')
  plot([4.49021,4.32925] ,[8.00227, 4.97177], 'k')
  plot([3.58095,4.32925] ,[4.83603, 4.97177], 'k')
  plot([3.58095,4.29435] ,[4.83603, 2.3476], 'k')
  plot([8.55028,8.4138] ,[2.1612, 2.49004], 'k')
  plot([8.55028,8.30271] ,[2.1612, 2.89608], 'k')
  plot([8.55028,9.38223] ,[2.1612, 3.33094], 'k')
  plot([8.55028,9.75162] ,[2.1612, 3.29726], 'k')
  plot([8.55028,7.90634] ,[2.1612, 3.71148], 'k')
  plot([8.55028,9.58847] ,[2.1612, 0.335065], 'k')
  plot([8.55028,9.07727] ,[2.1612, 4.56], 'k')
  plot([5.56811,5.83068] ,[8.88074, 8.13016], 'k')
  plot([5.56811,5.70059] ,[8.88074, 7.57113], 'k')
  plot([5.56811,6.25729] ,[8.88074, 5.51772], 'k')
  plot([3.87588,4.0214] ,[9.18491, 9.16122], 'k')
  plot([3.87588,3.61681] ,[9.18491, 8.86627], 'k')
  plot([3.87588,3.42228] ,[9.18491, 9.43566], 'k')
  plot([3.87588,3.41278] ,[9.18491, 9.93021], 'k')
  plot([3.87588,1.65688] ,[9.18491, 9.29783], 'k')
  plot([7.74701,7.98199] ,[9.85944, 8.3534], 'k')
  plot([7.74701,9.30954] ,[9.85944, 8.74511], 'k')
  plot([7.74701,5.56811] ,[9.85944, 8.88074], 'k')
  plot([7.74701,8.95791] ,[9.85944, 7.2662], 'k')
  plot([7.74701,9.00291] ,[9.85944, 6.61031], 'k')
  plot([2.11126,2.01093] ,[5.35839, 5.4666], 'k')
  plot([2.11126,1.6275] ,[5.35839, 6.69239], 'k')
  plot([2.11126,2.00757] ,[5.35839, 3.61369], 'k')
  plot([2.11126,1.70265] ,[5.35839, 7.221], 'k')
  plot([2.11126,2.252] ,[5.35839, 1.81952], 'k')
  plot([2.92855,3.42228] ,[9.28991, 9.43566], 'k')
  plot([2.92855,3.41278] ,[9.28991, 9.93021], 'k')
  plot([2.92855,3.87588] ,[9.28991, 9.18491], 'k')
  plot([2.92855,4.0214] ,[9.28991, 9.16122], 'k')
  plot([2.92855,1.65688] ,[9.28991, 9.29783], 'k')
  plot([7.29218,7.74701] ,[9.61765, 9.85944], 'k')
  plot([7.29218,7.98199] ,[9.61765, 8.3534], 'k')
  plot([7.29218,5.56811] ,[9.61765, 8.88074], 'k')
  plot([7.29218,9.30954] ,[9.61765, 8.74511], 'k')
  plot([7.29218,8.95791] ,[9.61765, 7.2662], 'k')
  plot([5.12358,4.36933] ,[0.873126, 0.675732], 'k')
  plot([5.12358,6.31133] ,[0.873126, 0.233989], 'k')
  plot([4.45368,4.49021] ,[8.29499, 8.00227], 'k')
  plot([4.45368,4.0214] ,[8.29499, 9.16122], 'k')
  plot([4.45368,3.61681] ,[8.29499, 8.86627], 'k')
  plot([4.45368,3.87588] ,[8.29499, 9.18491], 'k')
  plot([4.45368,3.42228] ,[8.29499, 9.43566], 'k')
  plot([4.45368,3.41278] ,[8.29499, 9.93021], 'k')
  plot([1.85228,1.70265] ,[7.61197, 7.221], 'k')
  plot([1.85228,1.6275] ,[7.61197, 6.69239], 'k')
  plot([1.85228,1.65688] ,[7.61197, 9.29783], 'k')
  plot([1.85228,2.01093] ,[7.61197, 5.4666], 'k')
  plot([1.85228,2.11126] ,[7.61197, 5.35839], 'k')
  plot([9.77317,9.75162] ,[2.64931, 3.29726], 'k')
  plot([9.77317,9.38223] ,[2.64931, 3.33094], 'k')
  plot([9.77317,8.55028] ,[2.64931, 2.1612], 'k')
  plot([9.77317,8.4138] ,[2.64931, 2.49004], 'k')
  plot([9.77317,8.30271] ,[2.64931, 2.89608], 'k')
  plot([9.77317,9.07727] ,[2.64931, 4.56], 'k')
  plot([9.77317,7.90634] ,[2.64931, 3.71148], 'k')
  plot([9.77317,9.58847] ,[2.64931, 0.335065], 'k')
  plot([9.77317,8.83602] ,[2.64931, 5.31897], 'k')
  plot([8.70446,8.83602] ,[5.23163, 5.31897], 'k')
  plot([8.70446,9.07727] ,[5.23163, 4.56], 'k')
  plot([8.70446,7.61138] ,[5.23163, 5.36579], 'k')
  plot([8.70446,9.00291] ,[5.23163, 6.61031], 'k')
  plot([8.70446,7.90634] ,[5.23163, 3.71148], 'k')
  plot([8.70446,9.38223] ,[5.23163, 3.33094], 'k')
  plot([8.70446,8.95791] ,[5.23163, 7.2662], 'k')
  plot([8.70446,9.75162] ,[5.23163, 3.29726], 'k')
  plot([8.70446,8.30271] ,[5.23163, 2.89608], 'k')
  plot([4.11237,4.36933] ,[0.965419, 0.675732], 'k')
  plot([4.11237,5.12358] ,[0.965419, 0.873126], 'k')
  plot([4.11237,4.29435] ,[0.965419, 2.3476], 'k')
  plot([4.11237,6.31133] ,[0.965419, 0.233989], 'k')
  plot([3.93729,3.61681] ,[8.09388, 8.86627], 'k')
  plot([3.93729,4.0214] ,[8.09388, 9.16122], 'k')
  plot([3.93729,3.87588] ,[8.09388, 9.18491], 'k')
  plot([3.93729,3.42228] ,[8.09388, 9.43566], 'k')
  plot([0.352578,0.477754] ,[6.86584, 6.3506], 'k')
  plot([4.28461,4.32925] ,[4.67598, 4.97177], 'k')
  plot([4.28461,3.58095] ,[4.67598, 4.83603], 'k')
  plot([4.28461,4.29435] ,[4.67598, 2.3476], 'k')
  plot([6.76277,7.29218] ,[9.40819, 9.61765], 'k')
  plot([6.76277,7.74701] ,[9.40819, 9.85944], 'k')
  plot([6.76277,5.56811] ,[9.40819, 8.88074], 'k')
  plot([6.76277,5.83068] ,[9.40819, 8.13016], 'k')
  plot([6.76277,9.30954] ,[9.40819, 8.74511], 'k')
  plot([5.5491,5.8601] ,[1.21645, 1.49491], 'k')
  plot([5.5491,5.99022] ,[1.21645, 1.67586], 'k')
  plot([5.5491,6.31133] ,[1.21645, 0.233989], 'k')
  plot([5.5491,5.98355] ,[1.21645, 2.92635], 'k')
  plot([5.5491,6.07014] ,[1.21645, 2.95894], 'k')
  plot([7.70318,7.98199] ,[7.40138, 8.3534], 'k')
  plot([7.70318,8.95791] ,[7.40138, 7.2662], 'k')
  plot([7.70318,9.00291] ,[7.40138, 6.61031], 'k')
  plot([7.70318,7.61138] ,[7.40138, 5.36579], 'k')
  plot([7.70318,9.30954] ,[7.40138, 8.74511], 'k')
  plot([7.70318,8.83602] ,[7.40138, 5.31897], 'k')
  plot([8.82842,8.70446] ,[4.87018, 5.23163], 'k')
  plot([8.82842,9.07727] ,[4.87018, 4.56], 'k')
  plot([8.82842,8.83602] ,[4.87018, 5.31897], 'k')
  plot([8.82842,7.61138] ,[4.87018, 5.36579], 'k')
  plot([8.82842,7.90634] ,[4.87018, 3.71148], 'k')
  plot([8.82842,9.38223] ,[4.87018, 3.33094], 'k')
  plot([8.82842,9.00291] ,[4.87018, 6.61031], 'k')
  plot([8.82842,9.75162] ,[4.87018, 3.29726], 'k')
  plot([8.82842,8.30271] ,[4.87018, 2.89608], 'k')
  plot([8.82842,8.95791] ,[4.87018, 7.2662], 'k')
  plot([7.80393,7.98199] ,[8.6016, 8.3534], 'k')
  plot([7.80393,7.29218] ,[8.6016, 9.61765], 'k')
  plot([7.80393,7.70318] ,[8.6016, 7.40138], 'k')
  plot([7.80393,7.74701] ,[8.6016, 9.85944], 'k')
  plot([7.80393,9.30954] ,[8.6016, 8.74511], 'k')
  plot([7.80393,8.95791] ,[8.6016, 7.2662], 'k')
  plot([7.80393,9.00291] ,[8.6016, 6.61031], 'k')
  plot([7.51949,7.70318] ,[6.50839, 7.40138], 'k')
  plot([7.51949,7.61138] ,[6.50839, 5.36579], 'k')
  plot([7.51949,9.00291] ,[6.50839, 6.61031], 'k')
  plot([7.51949,8.95791] ,[6.50839, 7.2662], 'k')
  plot([7.51949,8.70446] ,[6.50839, 5.23163], 'k')
  plot([7.51949,8.83602] ,[6.50839, 5.31897], 'k')
  plot([7.51949,7.98199] ,[6.50839, 8.3534], 'k')
  plot([7.51949,8.82842] ,[6.50839, 4.87018], 'k')
  plot([3.83323,3.58095] ,[4.60902, 4.83603], 'k')
  plot([3.83323,4.28461] ,[4.60902, 4.67598], 'k')
  plot([3.83323,4.32925] ,[4.60902, 4.97177], 'k')
  plot([3.83323,4.29435] ,[4.60902, 2.3476], 'k')
  plot([5.57444,5.82931] ,[4.23843, 4.63547], 'k')
  plot([5.57444,6.07014] ,[4.23843, 2.95894], 'k')
  plot([5.57444,5.98355] ,[4.23843, 2.92635], 'k')
  plot([5.57444,6.25729] ,[4.23843, 5.51772], 'k')
  plot([9.80497,9.00291] ,[6.3993, 6.61031], 'k')
  plot([9.80497,8.95791] ,[6.3993, 7.2662], 'k')
  plot([9.80497,8.83602] ,[6.3993, 5.31897], 'k')
  plot([9.80497,8.70446] ,[6.3993, 5.23163], 'k')
  plot([9.80497,8.82842] ,[6.3993, 4.87018], 'k')
  plot([9.80497,9.07727] ,[6.3993, 4.56], 'k')
  plot([9.80497,7.70318] ,[6.3993, 7.40138], 'k')
  plot([9.80497,9.30954] ,[6.3993, 8.74511], 'k')
  plot([9.80497,7.61138] ,[6.3993, 5.36579], 'k')
  plot([9.80717,9.80497] ,[7.02274, 6.3993], 'k')
  plot([9.80717,8.95791] ,[7.02274, 7.2662], 'k')
  plot([9.80717,9.00291] ,[7.02274, 6.61031], 'k')
  plot([9.80717,9.30954] ,[7.02274, 8.74511], 'k')
  plot([9.80717,8.83602] ,[7.02274, 5.31897], 'k')
  plot([9.80717,8.70446] ,[7.02274, 5.23163], 'k')
  plot([9.80717,7.70318] ,[7.02274, 7.40138], 'k')
  plot([9.80717,7.98199] ,[7.02274, 8.3534], 'k')
  plot([9.80717,8.82842] ,[7.02274, 4.87018], 'k')
  plot([2.44938,2.00757] ,[3.74445, 3.61369], 'k')
  plot([2.44938,2.11126] ,[3.74445, 5.35839], 'k')
  plot([2.44938,2.01093] ,[3.74445, 5.4666], 'k')
  plot([2.44938,2.252] ,[3.74445, 1.81952], 'k')
  plot([2.44938,1.6275] ,[3.74445, 6.69239], 'k')
  plot([9.94708,9.07727] ,[4.8949, 4.56], 'k')
  plot([9.94708,8.82842] ,[4.8949, 4.87018], 'k')
  plot([9.94708,8.83602] ,[4.8949, 5.31897], 'k')
  plot([9.94708,8.70446] ,[4.8949, 5.23163], 'k')
  plot([9.94708,9.80497] ,[4.8949, 6.3993], 'k')
  plot([9.94708,9.75162] ,[4.8949, 3.29726], 'k')
  plot([9.94708,9.38223] ,[4.8949, 3.33094], 'k')
  plot([9.94708,9.00291] ,[4.8949, 6.61031], 'k')
  plot([9.94708,9.80717] ,[4.8949, 7.02274], 'k')
  plot([9.94708,9.77317] ,[4.8949, 2.64931], 'k')
  plot([4.30309,4.29435] ,[2.7255, 2.3476], 'k')
  plot([4.30309,4.11237] ,[2.7255, 0.965419], 'k')
  plot([4.30309,3.83323] ,[2.7255, 4.60902], 'k')
  plot([4.30309,4.28461] ,[2.7255, 4.67598], 'k')
  plot([7.92629,8.55028] ,[2.00627, 2.1612], 'k')
  plot([7.92629,8.4138] ,[2.00627, 2.49004], 'k')
  plot([7.92629,8.30271] ,[2.00627, 2.89608], 'k')
  plot([7.92629,7.90634] ,[2.00627, 3.71148], 'k')
  plot([7.92629,7.51458] ,[2.00627, 0.125023], 'k')
  plot([7.92629,9.77317] ,[2.00627, 2.64931], 'k')
  plot([7.92629,9.38223] ,[2.00627, 3.33094], 'k')
  plot([0.12688,0.352578] ,[6.75472, 6.86584], 'k')
  plot([4.43921,4.0214] ,[9.18954, 9.16122], 'k')
  plot([4.43921,3.87588] ,[9.18954, 9.18491], 'k')
  plot([4.43921,3.61681] ,[9.18954, 8.86627], 'k')
  plot([4.43921,4.45368] ,[9.18954, 8.29499], 'k')
  plot([4.43921,3.42228] ,[9.18954, 9.43566], 'k')
  plot([4.43921,4.49021] ,[9.18954, 8.00227], 'k')
  plot([4.43921,3.93729] ,[9.18954, 8.09388], 'k')
  plot([4.43921,3.41278] ,[9.18954, 9.93021], 'k')
  plot([4.43921,2.92855] ,[9.18954, 9.28991], 'k')
  plot([9.00496,8.83602] ,[5.22016, 5.31897], 'k')
  plot([9.00496,8.70446] ,[5.22016, 5.23163], 'k')
  plot([9.00496,8.82842] ,[5.22016, 4.87018], 'k')
  plot([9.00496,9.07727] ,[5.22016, 4.56], 'k')
  plot([9.00496,9.94708] ,[5.22016, 4.8949], 'k')
  plot([9.00496,9.00291] ,[5.22016, 6.61031], 'k')
  plot([9.00496,7.61138] ,[5.22016, 5.36579], 'k')
  plot([9.00496,9.80497] ,[5.22016, 6.3993], 'k')
  plot([9.00496,7.90634] ,[5.22016, 3.71148], 'k')
  plot([9.00496,9.38223] ,[5.22016, 3.33094], 'k')
  plot([9.4586,9.80717] ,[6.94011, 7.02274], 'k')
  plot([9.4586,9.00291] ,[6.94011, 6.61031], 'k')
  plot([9.4586,8.95791] ,[6.94011, 7.2662], 'k')
  plot([9.4586,9.80497] ,[6.94011, 6.3993], 'k')
  plot([9.4586,8.83602] ,[6.94011, 5.31897], 'k')
  plot([9.4586,9.00496] ,[6.94011, 5.22016], 'k')
  plot([9.4586,9.30954] ,[6.94011, 8.74511], 'k')
  plot([9.4586,7.70318] ,[6.94011, 7.40138], 'k')
  plot([9.4586,8.70446] ,[6.94011, 5.23163], 'k')
  plot([3.96286,4.30309] ,[3.42808, 2.7255], 'k')
  plot([3.96286,4.29435] ,[3.42808, 2.3476], 'k')
  plot([3.96286,3.83323] ,[3.42808, 4.60902], 'k')
  plot([3.96286,4.28461] ,[3.42808, 4.67598], 'k')
  plot([3.96286,3.58095] ,[3.42808, 4.83603], 'k')
  plot([3.96286,4.32925] ,[3.42808, 4.97177], 'k')
  plot([3.01022,2.92855] ,[9.07948, 9.28991], 'k')
  plot([3.01022,3.42228] ,[9.07948, 9.43566], 'k')
  plot([3.01022,3.87588] ,[9.07948, 9.18491], 'k')
  plot([3.01022,3.41278] ,[9.07948, 9.93021], 'k')
  plot([3.01022,4.0214] ,[9.07948, 9.16122], 'k')
  plot([3.01022,1.65688] ,[9.07948, 9.29783], 'k')
  plot([3.01022,4.43921] ,[9.07948, 9.18954], 'k')
  plot([6.23004,6.07014] ,[3.62051, 2.95894], 'k')
  plot([6.23004,5.98355] ,[3.62051, 2.92635], 'k')
  plot([6.23004,5.57444] ,[3.62051, 4.23843], 'k')
  plot([6.23004,5.82931] ,[3.62051, 4.63547], 'k')
  plot([6.23004,6.25729] ,[3.62051, 5.51772], 'k')
  plot([6.23004,5.99022] ,[3.62051, 1.67586], 'k')
  plot([6.23004,5.8601] ,[3.62051, 1.49491], 'k')
  plot([1.5799,1.6275] ,[6.17711, 6.69239], 'k')
  plot([1.5799,2.01093] ,[6.17711, 5.4666], 'k')
  plot([1.5799,2.11126] ,[6.17711, 5.35839], 'k')
  plot([1.5799,1.70265] ,[6.17711, 7.221], 'k')
  plot([1.5799,1.85228] ,[6.17711, 7.61197], 'k')
  plot([1.5799,2.44938] ,[6.17711, 3.74445], 'k')
  plot([8.51541,7.98199] ,[8.7563, 8.3534], 'k')
  plot([8.51541,7.80393] ,[8.7563, 8.6016], 'k')
  plot([8.51541,9.30954] ,[8.7563, 8.74511], 'k')
  plot([8.51541,7.74701] ,[8.7563, 9.85944], 'k')
  plot([8.51541,7.29218] ,[8.7563, 9.61765], 'k')
  plot([8.51541,8.95791] ,[8.7563, 7.2662], 'k')
  plot([8.51541,7.70318] ,[8.7563, 7.40138], 'k')
  plot([8.51541,6.76277] ,[8.7563, 9.40819], 'k')
  plot([8.51541,9.4586] ,[8.7563, 6.94011], 'k')
  plot([8.51541,9.80717] ,[8.7563, 7.02274], 'k')
  plot([1.4818,2.252] ,[0.813248, 1.81952], 'k')
  plot([1.4818,0.172923] ,[0.813248, 0.680657], 'k')
  plot([1.4818,2.00757] ,[0.813248, 3.61369], 'k')
  plot([1.4818,2.44938] ,[0.813248, 3.74445], 'k')
  plot([7.56796,7.92629] ,[1.70122, 2.00627], 'k')
  plot([7.56796,8.55028] ,[1.70122, 2.1612], 'k')
  plot([7.56796,8.4138] ,[1.70122, 2.49004], 'k')
  plot([7.56796,8.30271] ,[1.70122, 2.89608], 'k')
  plot([7.56796,7.51458] ,[1.70122, 0.125023], 'k')
  plot([9.5395,9.77317] ,[2.92428, 2.64931], 'k')
  plot([9.5395,9.75162] ,[2.92428, 3.29726], 'k')
  plot([9.5395,9.38223] ,[2.92428, 3.33094], 'k')
  plot([9.5395,8.4138] ,[2.92428, 2.49004], 'k')
  plot([9.5395,8.30271] ,[2.92428, 2.89608], 'k')
  plot([9.5395,8.55028] ,[2.92428, 2.1612], 'k')
  plot([9.5395,9.07727] ,[2.92428, 4.56], 'k')
  plot([9.5395,7.90634] ,[2.92428, 3.71148], 'k')
  plot([9.5395,7.92629] ,[2.92428, 2.00627], 'k')
  plot([9.5395,9.94708] ,[2.92428, 4.8949], 'k')
  plot([6.09715,6.23004] ,[3.9787, 3.62051], 'k')
  plot([6.09715,5.57444] ,[3.9787, 4.23843], 'k')
  plot([6.09715,5.82931] ,[3.9787, 4.63547], 'k')
  plot([6.09715,6.07014] ,[3.9787, 2.95894], 'k')
  plot([6.09715,5.98355] ,[3.9787, 2.92635], 'k')
  plot([6.09715,6.25729] ,[3.9787, 5.51772], 'k')
  plot([2.11382,2.11126] ,[5.10211, 5.35839], 'k')
  plot([2.11382,2.01093] ,[5.10211, 5.4666], 'k')
  plot([2.11382,1.5799] ,[5.10211, 6.17711], 'k')
  plot([2.11382,2.44938] ,[5.10211, 3.74445], 'k')
  plot([2.11382,2.00757] ,[5.10211, 3.61369], 'k')
  plot([2.11382,1.6275] ,[5.10211, 6.69239], 'k')
  plot([2.11382,1.70265] ,[5.10211, 7.221], 'k')
  plot([9.19887,9.30954] ,[9.24896, 8.74511], 'k')
  plot([9.19887,8.51541] ,[9.24896, 8.7563], 'k')
  plot([9.19887,7.98199] ,[9.24896, 8.3534], 'k')
  plot([9.19887,7.80393] ,[9.24896, 8.6016], 'k')
  plot([9.19887,7.74701] ,[9.24896, 9.85944], 'k')
  plot([9.19887,7.29218] ,[9.24896, 9.61765], 'k')
  plot([9.19887,8.95791] ,[9.24896, 7.2662], 'k')
  plot([9.19887,9.80717] ,[9.24896, 7.02274], 'k')
  plot([9.19887,9.4586] ,[9.24896, 6.94011], 'k')
  plot([9.19887,7.70318] ,[9.24896, 7.40138], 'k')
  plot([9.68151,9.30954] ,[8.65746, 8.74511], 'k')
  plot([9.68151,9.19887] ,[8.65746, 9.24896], 'k')
  plot([9.68151,8.51541] ,[8.65746, 8.7563], 'k')
  plot([9.68151,8.95791] ,[8.65746, 7.2662], 'k')
  plot([9.68151,9.80717] ,[8.65746, 7.02274], 'k')
  plot([9.68151,7.98199] ,[8.65746, 8.3534], 'k')
  plot([9.68151,9.4586] ,[8.65746, 6.94011], 'k')
  plot([9.68151,7.80393] ,[8.65746, 8.6016], 'k')
  plot([9.68151,9.00291] ,[8.65746, 6.61031], 'k')
  plot([9.68151,9.80497] ,[8.65746, 6.3993], 'k')
  plot([6.18908,6.31133] ,[0.660208, 0.233989], 'k')
  plot([6.18908,5.5491] ,[0.660208, 1.21645], 'k')
  plot([6.18908,5.8601] ,[0.660208, 1.49491], 'k')
  plot([6.18908,5.99022] ,[0.660208, 1.67586], 'k')
  plot([6.18908,5.12358] ,[0.660208, 0.873126], 'k')
  plot([6.18908,4.36933] ,[0.660208, 0.675732], 'k')
  plot([6.18908,4.11237] ,[0.660208, 0.965419], 'k')
  plot([7.92323,7.56796] ,[0.151931, 1.70122], 'k')
  plot([7.92323,9.58847] ,[0.151931, 0.335065], 'k')
  plot([7.92323,7.92629] ,[0.151931, 2.00627], 'k')
  plot([7.92323,8.55028] ,[0.151931, 2.1612], 'k')
  plot([7.92323,8.4138] ,[0.151931, 2.49004], 'k')
  plot([4.08829,4.11237] ,[0.933444, 0.965419], 'k')
  plot([4.08829,4.36933] ,[0.933444, 0.675732], 'k')
  plot([4.08829,5.12358] ,[0.933444, 0.873126], 'k')
  plot([4.08829,4.29435] ,[0.933444, 2.3476], 'k')
  plot([4.08829,4.30309] ,[0.933444, 2.7255], 'k')
  plot([4.08829,6.18908] ,[0.933444, 0.660208], 'k')
  plot([9.23141,9.58847] ,[0.318324, 0.335065], 'k')
  plot([9.23141,7.92323] ,[0.318324, 0.151931], 'k')
  plot([9.23141,8.55028] ,[0.318324, 2.1612], 'k')
  plot([9.23141,7.92629] ,[0.318324, 2.00627], 'k')
  plot([9.23141,7.56796] ,[0.318324, 1.70122], 'k')
  plot([9.23141,8.4138] ,[0.318324, 2.49004], 'k')
  plot([9.23141,9.77317] ,[0.318324, 2.64931], 'k')
  plot([9.23141,9.5395] ,[0.318324, 2.92428], 'k')
  plot([9.23141,8.30271] ,[0.318324, 2.89608], 'k')
  plot([4.55396,4.36933] ,[0.811316, 0.675732], 'k')
  plot([4.55396,4.11237] ,[0.811316, 0.965419], 'k')
  plot([4.55396,4.08829] ,[0.811316, 0.933444], 'k')
  plot([4.55396,5.12358] ,[0.811316, 0.873126], 'k')
  plot([4.55396,6.18908] ,[0.811316, 0.660208], 'k')
  plot([4.55396,6.31133] ,[0.811316, 0.233989], 'k')
  plot([6.49544,6.07014] ,[3.06937, 2.95894], 'k')
  plot([6.49544,5.98355] ,[3.06937, 2.92635], 'k')
  plot([6.49544,6.23004] ,[3.06937, 3.62051], 'k')
  plot([6.49544,6.09715] ,[3.06937, 3.9787], 'k')
  plot([6.49544,5.99022] ,[3.06937, 1.67586], 'k')
  plot([6.49544,5.57444] ,[3.06937, 4.23843], 'k')
  plot([6.49544,5.8601] ,[3.06937, 1.49491], 'k')
  plot([6.49544,5.82931] ,[3.06937, 4.63547], 'k')
  plot([9.56762,9.19887] ,[9.38239, 9.24896], 'k')
  plot([9.56762,9.30954] ,[9.38239, 8.74511], 'k')
  plot([9.56762,9.68151] ,[9.38239, 8.65746], 'k')
  plot([9.56762,8.51541] ,[9.38239, 8.7563], 'k')
  plot([9.56762,7.74701] ,[9.38239, 9.85944], 'k')
  plot([9.56762,7.98199] ,[9.38239, 8.3534], 'k')
  plot([9.56762,7.80393] ,[9.38239, 8.6016], 'k')
  plot([9.56762,8.95791] ,[9.38239, 7.2662], 'k')
  plot([9.56762,7.29218] ,[9.38239, 9.61765], 'k')
  plot([9.56762,9.80717] ,[9.38239, 7.02274], 'k')
  plot([5.88787,5.5491] ,[1.04942, 1.21645], 'k')
  plot([5.88787,5.8601] ,[1.04942, 1.49491], 'k')
  plot([5.88787,6.18908] ,[1.04942, 0.660208], 'k')
  plot([5.88787,5.99022] ,[1.04942, 1.67586], 'k')
  plot([5.88787,5.12358] ,[1.04942, 0.873126], 'k')
  plot([5.88787,6.31133] ,[1.04942, 0.233989], 'k')
  plot([5.88787,4.55396] ,[1.04942, 0.811316], 'k')
  plot([5.88787,4.36933] ,[1.04942, 0.675732], 'k')
  plot([0.19564,0.172923] ,[0.712646, 0.680657], 'k')
  plot([0.19564,1.4818] ,[0.712646, 0.813248], 'k')
  plot([2.41386,2.252] ,[2.1976, 1.81952], 'k')
  plot([2.41386,2.00757] ,[2.1976, 3.61369], 'k')
  plot([2.41386,2.44938] ,[2.1976, 3.74445], 'k')
  plot([2.41386,1.4818] ,[2.1976, 0.813248], 'k')
  plot([6.17631,6.07014] ,[2.8017, 2.95894], 'k')
  plot([6.17631,5.98355] ,[2.8017, 2.92635], 'k')
  plot([6.17631,6.23004] ,[2.8017, 3.62051], 'k')
  plot([6.17631,5.99022] ,[2.8017, 1.67586], 'k')
  plot([6.17631,6.09715] ,[2.8017, 3.9787], 'k')
  plot([6.17631,5.8601] ,[2.8017, 1.49491], 'k')
  plot([6.17631,5.57444] ,[2.8017, 4.23843], 'k')
  plot([6.17631,5.5491] ,[2.8017, 1.21645], 'k')
  plot([3.61313,3.58095] ,[5.37517, 4.83603], 'k')
  plot([3.61313,3.83323] ,[5.37517, 4.60902], 'k')
  plot([3.61313,4.32925] ,[5.37517, 4.97177], 'k')
  plot([3.61313,4.28461] ,[5.37517, 4.67598], 'k')
  plot([3.61313,3.96286] ,[5.37517, 3.42808], 'k')
  plot([2.05066,2.00757] ,[3.29464, 3.61369], 'k')
  plot([2.05066,2.44938] ,[3.29464, 3.74445], 'k')
  plot([2.05066,2.41386] ,[3.29464, 2.1976], 'k')
  plot([2.05066,2.252] ,[3.29464, 1.81952], 'k')
  plot([2.05066,2.11382] ,[3.29464, 5.10211], 'k')
  plot([2.05066,2.11126] ,[3.29464, 5.35839], 'k')
  plot([2.05066,2.01093] ,[3.29464, 5.4666], 'k')
  plot([4.03264,3.93729] ,[8.23974, 8.09388], 'k')
  plot([4.03264,3.61681] ,[8.23974, 8.86627], 'k')
  plot([4.03264,4.0214] ,[8.23974, 9.16122], 'k')
  plot([4.03264,3.87588] ,[8.23974, 9.18491], 'k')
  plot([4.03264,4.43921] ,[8.23974, 9.18954], 'k')
  plot([4.03264,3.42228] ,[8.23974, 9.43566], 'k')
end
plot([0.5, 0.19564], [0.5, 0.712646], 'r', 'LineWidth', 2)
plot([0.19564, 1.4818], [0.712646, 0.813248], 'r', 'LineWidth', 2)
plot([1.4818, 2.00757], [0.813248, 3.61369], 'r', 'LineWidth', 2)
plot([2.00757, 1.65688], [3.61369, 9.29783], 'r', 'LineWidth', 2)
plot([1.65688, 3.42228], [9.29783, 9.43566], 'r', 'LineWidth', 2)
plot([3.42228, 4.32925], [9.43566, 4.97177], 'r', 'LineWidth', 2)
plot([4.32925, 4.29435], [4.97177, 2.3476], 'r', 'LineWidth', 2)
plot([4.29435, 4.11237], [2.3476, 0.965419], 'r', 'LineWidth', 2)
plot([4.11237, 5.12358], [0.965419, 0.873126], 'r', 'LineWidth', 2)
plot([5.12358, 5.88787], [0.873126, 1.04942], 'r', 'LineWidth', 2)
plot([5.88787, 5.8601], [1.04942, 1.49491], 'r', 'LineWidth', 2)
plot([5.8601, 5.83068], [1.49491, 8.13016], 'r', 'LineWidth', 2)
plot([5.83068, 6.76277], [8.13016, 9.40819], 'r', 'LineWidth', 2)
plot([6.76277, 7.29218], [9.40819, 9.61765], 'r', 'LineWidth', 2)
plot([7.29218, 9.19887], [9.61765, 9.24896], 'r', 'LineWidth', 2)
plot([9.19887, 9], [9.24896, 9], 'r', 'LineWidth', 2)
text(0.5, 0.5,'S','FontSize',12)
text(9, 9,'G','FontSize',12)
hold off
title('Number of nodes is 100 and K (number of edge attempts) is 10')
axis equal