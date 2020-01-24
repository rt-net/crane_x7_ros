<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ジャイロの値を得る</title>
</head>
 
<body>
<div id="txt">ここにデータを表示</div>             <!-- データを表示するdiv要素 -->
 
<script>
var alpha = 0, beta = 0, gamma = 0;             // ジャイロの値を入れる変数を3個用意
 
// ジャイロセンサの値が変化したら実行される deviceorientation イベント
window.addEventListener("deviceorientation", (dat) => {
    alpha = dat.alpha;  // z軸（表裏）まわりの回転の角度（反時計回りがプラス）
    beta  = dat.beta;   // x軸（左右）まわりの回転の角度（引き起こすとプラス）
    gamma = dat.gamma;  // y軸（上下）まわりの回転の角度（右に傾けるとプラス）
});
 
// 指定時間ごとに繰り返し実行される setInterval(実行する内容, 間隔[ms]) タイマーを設定
var timer = window.setInterval(() => {
    displayData();      // displayData 関数を実行
}, 33); // 33msごとに（1秒間に約30回）
 
// データを表示する displayData 関数
function displayData() {
    var txt = document.getElementById("txt");   // データを表示するdiv要素の取得
    txt.innerHTML = "alpha: " + alpha + "<br>"  // x軸の値
                  + "beta:  " + beta  + "<br>"  // y軸の値
                  + "gamma: " + gamma;          // z軸の値
}
</script>
</body>
</html>