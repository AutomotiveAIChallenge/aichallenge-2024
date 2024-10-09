# aichallenge-2024
## how to install
```bash
git clone --recursive git@github.com:iASL-Gifu/aichallenge-2024-final.git
# if use only clone
git submodule update --init --recursive
```

## Toolの使用方法
**WIP**

## aichallengeについて
本リポジトリでは、2024年度に実施される自動運転AIチャレンジでご利用いただく開発環境を提供します。参加者の皆様には、Autoware Universe をベースとした自動運転ソフトウェアを開発し、予選大会では End to End シミュレーション空間を走行するレーシングカートにインテグレートしていただきます。開発した自動運転ソフトウェアで、安全に走行しながらタイムアタックに勝利することが目標です。また、決勝大会では本物のレーシングカートへのインテグレーションを行っていただきます。

This repository provides a development environment use in the Automotive AI Challenge which will be held in 2024. For the preliminaries, participants will develop autonomous driving software based on Autoware Universe and integrate it into a racing kart that drives in the End to End simulation space. The goal is to win in time attack while driving safely with the developed autonomous driving software. Also, for the finals, qualifiers will integrate it into a real racing kart.

## ドキュメント / Documentation

下記ページにて、本大会に関する情報 (ルールの詳細や環境構築方法) を提供する予定です。ご確認の上、奮って大会へご参加ください。

Toward the competition, we will update the following pages to provide information such as rules and how to set up your dev environment. Please follow them. We are looking forward your participation!

- [日本語ページ](https://automotiveaichallenge.github.io/aichallenge-documentation-2024/)
- [English Page](https://automotiveaichallenge.github.io/aichallenge-documentation-2024/en/)

## エラー対処
```bash
ros2: failed to increase socket receive buffer size to at least 10485760 bytes, current is 425984 bytes
[ERROR] [1728286982.310247407] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Error
```
```bash
sudo sysctl -w net.core.rmem_max=10485760
sudo sysctl -w net.core.rmem_default=10485760
```
