## 概要

マイクで録音/配信するツール群

## launchの使い方

- capture_to_file.launch

その名の通り録音したのは、fileに保存するツール

- capture.launch

GStreamer という、ストリーミングメディアアプリケーションを作成するためのフレームワークを使っている。
audio_playパッケージの、play.launchとの組み合わせによって、リアルタイムのストリーミングが可能
audio play のほうが、clientなどで、後から立ち上げる必要があり。
つなぎ先は選べるか不明。

- capture_wave.launch

不明。
