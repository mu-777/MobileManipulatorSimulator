% 固定点探索プログラム 20140724
%
%   １，設計方針
%       基本的に出来るものはオブジェクト化して，各責任が独立するようにした
%       しかしながら，クラスを使えることに気付くのが遅かったので，運動方程式などの部分は関数ベースになっている・・・・いつか変更したい
%
%       基本的な構造は3つ
%
%       A simulation_parameterクラス
%          　シミュレーションに必要なパラメタを管理するクラス
%            新しいパラメタを増やしたくなったらこのクラスに増やしてくれ   
%            
%       B simulation_simulatorクラス
%           解探索のシミュレーションを行って結果を返すクラス
%           探索すべき非線形方程式を参照する　なので，非線形方程式を変えたくなったらこの参照関数だけをいじってくれ
%           解探索のやり方（Newtonとかなんとか）を変えたくなったらこのクラスをいじってくれ
%
%       C　simulation_prameterManagerクラス
%           シミュレータのパラメタ変化に対応するクラス
%           パラメタの変化させ方を変えたくなったら，このクラスをいじってくれ
% 
%       D メイン関数
%           シミュレーションの構造を定義，現状3階層のループになっている　
%           ログ用の変数を管理している　（もちろん A-Cのオブジェクトも管理）
%
%   ２，大きな変化させたい時の方針          
%       運動方程式を変えたい　－＞　非線形方程式を変化させる（大きな変更，状態量が変わるとか・・・であればその依存関数も変化すべし），パラメタも変わるのであれば，Aも変更
%       simDatの中身を増やしたい -> Bのクラスを変更
%       パラメタの変化の仕方を変えたい（2つ一気に変えるとかなんとか） -> Cのクラスをいじる
%       
% 気付いた注意点など
%   MATLABのイベント判定の部分には，バグがあって，接地を見つけれないことがある，これによって，解探索がうまくいかない場合も出てくる
%   解探索で得られる解が，完全に固定点になっていない時，Jacobianの計算に１０＾１０オーダーの誤差が載ってしまうことがある
%   さらに，よく分からないが，MATLABのイベントを見つけれないバグの影響で，解が見つからないときや，安定性を評価できないときが存在する（今はこのような解は捨てている）
%   どうやら，MATLABで解が見つかりにくい理由は，2つのイベントが同時に起きているからくさい．．．．どうせ解の精度以下の誤差は入れても入れまいが一緒だから，解の精度以下の位相差の誤差1e-9をぶち込んで計算処理を軽くして，バグもなくす
%   　　上の記述は下の問題を解決したら消えた．．．．
%   さらに，初期状態でポアンカレ断面判定していると，その後上手くいかなくなるみたいな，イベントを判定したあとすぐにイベントが起こることは問題らしい．解決
%   正直よく分からない，イベント処理系の中にいろんな問題があると考えておいた方が良いだろう．結局，精度とかは，これまで通りにしておくのがベストな結果（イベントを判定できる）を作ってくれるらしい．
%   解探索はある程度は細かく行った方がよいでしょう！逆にその方がスピードが速い場合がある．
%   
%
%
% odeParam : 
%   シミュレート最大時間 maxSimTime　
%   シミュレーション精度関連 accuracy, times_of_refine
%
% robotParam : 
%   L, I, k
%
% simParam :
%   fixedPointAcc : 固定点の精度
%   jacobianAcc   : ヤコビアンの計算精度
%
%
% 扱っていく固定点
%   x_p =  [y theta dx dtheta g1 g2 g3 g4]
%
% データログ用変数 logDat
%   timedStatus: 時間による状態量などの変化ログ 
%     time(N) : 時間のベクトル  
%     x(N, 6) : 状態量のベクトル: [x y theta dx dy dtheta] 
%     legAngles : 脚と胴体の間の角度　[g1 g2 g3 g4] 
%     legLengthes : 足の長さ [l1 l2 l3 l4]
%     legContacts: 接地のあるなし[leg1 leg2 leg3 leg4] 1で接地している，０で浮いている
%
%   timedEvent: イベントによる状態量などの変化ログ
%     time(event num) : 時間ベクトル
%     x(event num, 6) : 状態量のベクトル [x y theta1 dx dy dtheta]    
%     ie() : デバッグ用  順番に,leg1-4の[脚の長さの判別，離地脚のy座標の判別]，y座標，dy  
%
%   liftOffAngles: 4つのリフトオフアングル  
%
% シミュレーションデータ　simDat
% simDat
%       findFlag : 1 or 0
%       fixedPoint_p : x_p
%       accuracy : 実際の解の精度
%       robotParam : 
%       odeParam : 
%       simParam : 
%       stabDat : 
%       analDat :
%
%
% stabDat : 
%  stabDat.eigVal = [eigVal1, ....] 
%  stabDat.eigVec(i) = eigVal(i)の横ベクトル
%  stabDat.maxEig : 保存則に対応する固有値1を除いた固有値
%  stabDat.error : 計算エラーが発生した
%
% analDat :
%  analDat.gaits  : [0x0000, 0x1000, ........; time, time2] (後ろから順に接地脚の接地１，離地０)
%  analDat.energy : エネルギー
%  analDat.Fr : フルド数
%  analDat.period : 周期
%  analDat.reactionMax : [leg1, leg2, leg3, leg4, maxOfThem] 床反力の大きさ
%  analDat.forcesMax : [f1x, f1y; f2x, f2y; f3x, f3y; f4x, f4y; fmaxx, fmaxy;]　床反力の成分
%  analDat.freq : 1周期の周波数
%  analDat.phases : [leg1_td, leg2_td, leg3_td, leg4_td; 
%                        leg1_lf, leg2_lf, leg3_lf, leg4_lf]　(0-1で規格化)
%  analDat.phaseDiffs : [前左右の位相差td(2-1)，後ろ左右の位相差td(4-3), 前後の位相差td(2-4), 前後の位相差td(1-3); , 前左右の位相差lf(2-1)，後ろ左右の位相差lf(4-3), 前後の位相差lf(2-4), 前後の位相差lf(1-3)]
%  analDat.legSpeed : [leg1_spd, leg2_spd, leg3_spd, leg4_spd, mean] 平均速度　[rad]/time (無次元量) 
%  analDat.duty : [leg1_duty, leg2_duty, leg3_duty, leg4_duty, mean] 
%  analDat.COMamp  : 重心の振幅 
%
%
%
%

%% 初期化および各摂動の設定

clear
close all force

%% Simulatorのパラメタ設定
%ODE param
 % ode45　のデータ出力数？？ グラフが粗ければ大きくしてくれ(拘束化のために1にしてるけど)
 odeParam.times_of_refine = 4;
 % 相対誤差(この値でないと同時接地を見抜けない)
 odeParam.relativeAccuracy = 1e-11; %基本的にこの値はいじらない方が良い　初期高さ１でOKにしたいなら 3-14
 % 絶対誤差
 odeParam.absoluteAccuracy = 1e-10; %基本的にこの値はいじらない方が良い 初期高さ１でOKにしたいなら 3-13
 % シミュレーション最大時間 [normalized]
 odeParam.maxSimulationTime = 10;   

%Simulation parameter
 % fixed point accuracy
 simParam.fixedPointAcc = 1e-7;% これくらいで実用上は妥当でしょう
 % jacobian accuracy
 simParam.jacobianAcc = 1e-6; % これも実用上は妥当でしょう
 
%解の探索関数 設定
% pFuncGallop = @(x, robotParam, odeParam, ctrParam, iniStatConst, otherConst) func_poincreMapGallop_with_constraints_Fr_y_dth_dgs(x, robotParam, odeParam, ctrParam, iniStatConst, otherConst);
pFuncGallop = @(x, robotParam, odeParam, ctrParam, iniStatConst, otherConst) func_poincreMapGallop_with_constraints_dx_y_dth_dg_scissorsym(x, robotParam, odeParam, ctrParam, iniStatConst, otherConst);

% これがシミュレーションで使われる！！
% Simulatorオブジェクトの作成
% *********************************************************************************************************************************
simulator = simulation_simulator(odeParam, simParam, pFuncGallop); 


%% Simulationにおけるパラメタ設定　(初期値とかロボット変数の値とか)
%Robot Param
% Lynx Param
x_ini_p = [0. 0.324 0. 1.39 0. (145.9/180.)* pi];
[robotParam.I, robotParam.L, robotParam.k, x_ini] = conv_normParam(7., 0.2015, 0.376/2., 1733.8, 0.22, x_ini_p);
robotParam.k = 5.56;
%robotParam.k = 6;
%robotParam.k = 7;
%robotParam.k = 5;
%robotParam.k = 4;

% 解の初期推定値設定
%x_p_ini = [1.10000000000000, 0., 0., 0., 0., 0., 0., 0.];%[1,6.73529538680675e-20,1.43881089646273,1.55743331943719e-20,0.413523522801169,0.413523522801169,0.413523522801169,0.413523522801169];
x_p_ini = [1.10000000000000,-3.38539965491732e-12,0.600000000000000,1.23257488903669e-17,0.216792079766339,0.216792069766339,0.216792079751332,0.216792069751332];%[1,6.73529538680675e-20,1.43881089646273,1.55743331943719e-20,0.413523522801169,0.413523522801169,0.413523522801169,0.413523522801169];
%x_p_ini = [0.999999999847460,-9.644265966710694e-11,1.440749540262108,1.499999999920854,0.441259863191393,0.441259863191393,0.490148494387054,0.490148494387054];
%x_p_ini = [x_ini(2), x_ini(3), x_ini(4), x_ini(6), 0.2, 0.2, 0.2, 0.2];

% あるいは保存されている，シミュレーション結果から読んでくる？？
%load('141214_initialEst_gallop_dg045.mat')

% 解の初期状態に対する拘束設定
iniStatConst = [1.1, 0., 0.6, 0., 0., 0., 0., 0.];

% 解のそのほかの拘束設定
%   dgf,dgbは完全に0にしたいが，そうするとMATLABがイベントを判別できない場合が存在するので，おそらく精度の問題だと思うが，解以下の精度にしておく
%     <- ポアンカレ断面判定しないようにしたら解決した！！　拘束値０にすべき 1e-9
%   others :  [dgf dgb Fr Energy]
otherConst = [1e-9, 1e-9, 0., 0.];


% これがシミュレーションで使われる！！
% parameter オブジェクトの作成
% *********************************************************************************************************************************
parameter = simulation_parameter(robotParam, x_p_ini, iniStatConst, otherConst);


%% Simulation におけるパラメタ変化のやり方の指定

% 最下層ループのパラメタ変化オブジェクト
%  simulation_prmChangeMng(changePrmName_, initialVal_, minVal_, maxVal_, step_, isUpdateIniGuess_, terminateWay_)
%     changePrmName_ :  初期推定値の8個 iniGuess_y,_th,_dx,_dth,_g1,_g2,_g3,_g4
%                       ポアンカレ断面の拘束値　iniStatus_y,_th,_dx,_dth,_g1,_g2,_g3,_g4
%                       ロボットのパラメタ　k, L, I
%                       その他の拘束値　dgf(前足の接地角度差), dgb, Fr(フルド数), E(エネルギー)
%     initialVal_：初期値, minVal_:探索最小値, maxVal_：探索最大値, step_：探索ステップ, isUpdateIniGuess_：初期推定値を自動的に前回の固定点を使う1か否0か, terminateWay_= 1:探索最大最小値まで探索 or 2:解がみつからなくなったら終わり                 

% dthに関しては、できるだけ細かくやるのがよい（角度で0.5度、0.005[rad]くらい？？）
%parameterManager1 = simulation_prmChangeMng('iniStatus_dth', 0., -0.001, 1.5, 0.005, 1, 1);
%parameterManager1 = simulation_prmChangeMng('iniStatus_dx', 0.6, 0.6, 2.4, 0.02, 1, 1);
parameterManager1 = simulation_prmChangeMng('iniStatus_dth', 0., 0., 5., 0.05, 1, 1);
waitbarNum(1) = parameterManager1.get_simRepNum; % どれくらいの個数が総じてあるか教えてあげれば進みが分かる 
% ２層ループのパラメタ変化オブジェクト
parameterManager2 = simulation_prmChangeMng('iniStatus_dx', 0.6, 0.0, 5., 0.2, 1, 1);
waitbarNum(2) = parameterManager2.get_simRepNum; % どれくらいの個数が総じてあるか教えてあげれば進みが分かる
% ３層ループのパラメタ変化オブジェクト
parameterManager3 = simulation_prmChangeMng('dgf', 0., 0., 1.0, 0.05, 1, 1);
waitbarNum(3) = parameterManager3.get_simRepNum; % どれくらいの個数が総じてあるか教えてあげれば進みが分かる


% これがシミュレーションで使われる！！
% 実際の実行関数にレジスト
parameterChanger_low = @(isSuccess, prvFixedPoint, parameter) parameterManager1.work(isSuccess, prvFixedPoint, parameter);
parameterChanger_mid = @(isSuccess, prvFixedPoint, parameter) parameterManager2.work(isSuccess, prvFixedPoint, parameter);
parameterChanger_high = @(isSuccess, prvFixedPoint, parameter) parameterManager3.work(isSuccess, prvFixedPoint, parameter);

%% Simulation の実行
% 基本的にこの部分はいじる必要はないはず！！
% 勝手にシミュレーションを回してくれる部分，初期値とかそういうのはオブジェクトを初期化するところで変えてくれ
% メモ：基本的に再起関数とか使ってオブジェクト化できたはずだけどやっていない！！　まあ今後余裕あれば

tic;
time = 0;
max = 0;
strings = {'よっしゃあ，はじめるでぇぇ', 'あっ，一応計算してるIDと', '残り時間は教えてあげるネ','','**********************************************************************'};
h = waitbar(0, strings);

% 各ループで限界回す数（特に問題なければ大きく設定して）
maxLoop = 100000;
% ログ用変数の定義
% LogSimDat(1,1,1);

% ループで使う変数の定義
 % 前回のsimulationで解が見つかったか否か 順に最下層，中層，最上
 isSuccess = [0, 0, 0];
 % 前回見つかった解 順に最下層，中層，最上
 prvFixedPoint = zeros(3,8);

%最上層ループ
for i = 1:maxLoop
    % パラメタ変更
    if parameterChanger_high(isSuccess(3), prvFixedPoint(3, :), parameter);
        % 終わりと判定された場合
        prvFixedPoint(3, :) = zeros(1, 8);
        isSuccess(3) = 0;
        break;
    end
    
    % 中層ループ
    for j = 1:maxLoop
        % パラメタ変更
        if parameterChanger_mid(isSuccess(2), prvFixedPoint(2, :), parameter);
            % 終わりと判定された場合
            prvFixedPoint(2, :) = zeros(1, 8);
            isSuccess(2) = 0;
            break;
        end
        
        % 最下層ループ
        for k = 1:maxLoop
            % パラメタ変更
            if parameterChanger_low(isSuccess(1), prvFixedPoint(1, :), parameter);
                % 終わりと判定された場合
                prvFixedPoint(1, :) = zeros(1, 8);
                isSuccess(1) = 0;
                break;
            end
            % シミュレーション実行
            simDat = simulator.work(parameter);
            % 値更新
            isSuccess(1) = simDat.findFlag;
            prvFixedPoint(1,:) = simDat.fixedPoint_p;
            % 上のループのための値更新
            if k == 1
                isSuccess(2) = isSuccess(1);
                prvFixedPoint(2,:) = prvFixedPoint(1,:);
            end
            % 結果のログ
            logSimDat(i).at(j,k) = simDat;
            
            % 状況を描画
            duration = toc - time;
            if duration > max
                max = duration;
            end
            time = toc;
            mean = time / (((i-1)*waitbarNum(1)*waitbarNum(2) + (j-1)*waitbarNum(1) + k));
            st = strcat('(i, j, k) =  (', num2str(i), ', ', num2str(j), ', ', num2str(k),')' );
            progress = ((i-1)*waitbarNum(1)*waitbarNum(2) + (j-1)*waitbarNum(1) + k) / (waitbarNum(1)*waitbarNum(2)*waitbarNum(3));
            remainTime = time * (1-progress)/progress;
            st3 = strcat('予想平均所要時間：', int2str(remainTime / 60), '分');
            st2 = strcat('今回のステップ所要時間：', int2str(duration), '[s]，平均ステップ所要時間:', int2str(mean), '[s], 最大ステップ所要時間:', int2str(max),'[s]');
            st4 = strcat('予想最大所要時間：', int2str(max * ((waitbarNum(1)*waitbarNum(2)*waitbarNum(3))-((i-1)*waitbarNum(1)*waitbarNum(2) + (j-1)*waitbarNum(1) + k))/60), '分');
            if progress < 0.5 
                strings = {'まだやってゆうてんねん・・・まあ，気長に待てや', st, st2, st3, st4};
            elseif progress < 0.8
                strings = {'半分はこえたでぇ・・・まあ，待ってて', st, st2, st3, st4};
            elseif progress < 0.9
                strings = {'もうちょいやでぇ・・・', st, st2, st3, st4};
            else 
                strings = {'もういける！・・・ヒヒヒ', st, st2, st3, st4};
            end
            waitbar(progress, h, strings);
        end
        
        % 上のループのための値更新
        if j == 1
            isSuccess(3) = isSuccess(2);
            prvFixedPoint(3,:) = prvFixedPoint(2,:);
        end        
    end
    % 上のループのための値更新 今回はなし 
end

time = toc;
st = strcat('計算時間 : ', int2str(time/60), '分');
strings = {'えっと．．．．もう終わってんねんけど・・・・・', st};
waitbar( 1, h, strings);

%% Data を　保存
description = {'Lynx：プロンクからバウンド、そしてギャロップの出現 y = 1.1 K = 5.56','iniStatus_dth, 0., 0., 5., 0.05, 1, 1','iniStatus_dx, 0.6, 0.0, 5., 0.2, 1, 1','dgf, 0., 0., 1., 0.05, 1, 1','a'};
save('141219_Gallop_PrmAnal_L085_I059_K556_SimPrm_dx_dth_dgf_Const_dx_y11_dth_dg_scissorsym.mat', 'logSimDat', 'description');

%% 結果の分析部分
close all

% 分析したい奴の番号
xNum = 3;
yNum = 1;
zNum = 1;

% 3次元グラフで分析
%plot_logSimData_3d(logSimDat(zNum).at, 'dth', 'dx', 'maxEigVal', [yNum, xNum]);
plot_logSimData_3d(logSimDat(zNum).at, 'dth', 'dx', 'g4', [yNum, xNum]);
plot_logSimData_3d_Zs(logSimDat(zNum).at, 'dth', 'dx', {'duty1', 'duty2', 'duty3', 'duty4', 'duty'}, [yNum, xNum], 'dutys');
plot_logSimData_3d_Zs(logSimDat(zNum).at, 'dth', 'dx', {'g1', 'g2', 'g3', 'g4'}, [yNum, xNum], 'gs');
plot_logSimData_3d_Zs(logSimDat(zNum).at, 'dth', 'dx', {'legSpeed1', 'legSpeed2', 'legSpeed3', 'legSpeed4', 'legSpeed'}, [yNum, xNum], 'legSpeeds');
%plot_logSimData_3d(logSimDat(zNum).at, 'dx', 'g3', 'y', [yNum, xNum]);

% 2次元で解析
%plot_logSimData_2d(logSimDat(zNum).at(yNum, :), 'dx', 'g1',  xNum);
plot_logSimData_2d(logSimDat(zNum).at(yNum, :), 'dth', 'g4',  xNum);
plot_logSimData_stability(logSimDat(zNum).at(yNum, :), 'dth', xNum);

% 2次元グラフで解析
%plot_logSimData_2d(logSimDat(zNum).at(:, xNum), 'dx', 'g1', yNum);
%plot_logSimData_2d(logSimDat(zNum).at(:, xNum), 'dx', 'g3', yNum);
%plot_logSimData_2d(logSimDat(1).at(:, xNum), 'dx', 'maxEigVal', yNum);

% 分析したいところのもの取り出し
simDat = logSimDat(zNum).at(yNum, xNum);

% simDat から logDatの作成
simDat.odeParam.times_of_refine = 4;
logDat = calc_logDat(simDat);
% Investigate result 
plot_data(logDat, simDat.robotParam, 0);
% make a movie
%visualize_data(logDat, simDat.robotParam, 100, 0.5, 0, 'timed_dth045_dx040_with_dg090.avi');

% 初期推定値を保存
robotParam = simDat.robotParam
x_p_ini = simDat.fixedPoint_p
iniStatConst = x_p_ini;
otherConst = [x_p_ini(5)-x_p_ini(6), x_p_ini(7)-x_p_ini(8), simDat.analDat.Fr, simDat.analDat.energy];
%save('141214_initialEst_gallop_dg045.mat', 'robotParam', 'x_p_ini', 'iniStatConst', 'otherConst');




