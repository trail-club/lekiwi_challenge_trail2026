# チーム開発ガイド

**対象:** LeKiwi Pick & Place Competition に参加する 1 チーム (7 名程度) 向け。
**目的:** 短期間で `pick / place / navigation` を統合し、大会ルール
([competition_rulebook.md](competition_rulebook.md)) で得点を最大化するための
開発フローと役割分担のリファレンス。

---

## 目次

1. [環境構築](#1-環境構築)
2. [開発フロー例](#2-開発フロー例)
3. [実装計画の作成](#3-実装計画の作成)
4. [担当割り振り例](#4-担当割り振り例-7-名想定)
5. [統合とテスト](#5-統合とテスト)

---

## 1. 環境構築

### 1.1 チームリポジトリの作成

上流リポジトリ `trail_SO101` を **チームリーダーが Fork** し、
チーム全員がそこへ Push できる形にする。

```
upstream:  github.com/trail-club/trail_SO101         (本家 / 大会運営)
   │  fork
   ▼
team:      github.com/trail-club/trail_SO101_<team_name>    (チーム共通レポ)
   │  clone
   ▼
member:    ~/dev/trail_SO101                    (各メンバの手元)
```

**手順**

1. リーダーが GitHub 上で Fork し、`trail-club` 配下に
   置く（チームごとにrenameする）。メンバー全員に `Write` 権限を付与する。
2. 各メンバは自分の PC に clone する（個人のコード修正用）。
   ```bash
   git clone git@github.com:<repository_name>.git
   cd trail_SO101
   git remote add upstream git@github.com:trail-club/trail_SO101.git
   ```
3. `upstream` から定期的に fetch し、ルールブックや基盤コードの更新を
   取り込む。
   ```bash
   git fetch upstream
   git merge upstream/main   # または rebase
   ```
4. `hsr-pc`にチームディレクトリを作成し、cloneする。上記設定を行う。以後実機の動作はこのディレクトリで行う。

### 1.2 ローカル開発環境

README (`README.md`) の 1〜5 章に従って、以下を`hsr-pc`のチームディレクトリで行う。

- Docker イメージのビルド (`make build`)
- udev ルールの導入、アーム較正
- ワークスペース初期化 (`colcon build`)

### 1.3 ブランチ運用例

- `main` — 常にビルド & 起動可能な安定版。直接 push 禁止。
- `dev/<feature>` — 機能開発ブランチ (例: `dev/perception-yolo`)。
- Pull Request で `main` にマージ。**最低 1 名のレビュー** を必須とする。
- 大会当日は `release/<date>` タグを切り、当日はそれを使う。

---

## 2. 開発フロー例

1. **週次スプリント (1 週間単位)** を回す。日曜に進捗報告を行う。
2. GitHub Issues で全タスクを管理。Label で機能領域を分けるなど
   (`perception`, `navigation`, `manipulation`, `integration`, `infra`)。
3. PR には **動作確認方法** をなるべく添付。

---

## 3. 実装計画の作成

大会タスクを **機能モジュール** に分割し、それぞれの依存関係・
マイルストーンを明確にする。以下は例。

### 3.1 モジュール分割例

| ID | モジュール | 主機能 | ROS 2 インタフェース例 | 担当領域 |
|---|---|---|---|---|
| M1 | **Mapping / Localization** | 事前マップ作成、AMCL による自己位置推定 | `/map`, `/amcl_pose` | Navigation |
| M2 | **Navigation** | Nav2 スタックで指定座標へ移動 | Action `NavigateToPose` | Navigation |
| M3 | **Exploration** | 未知位置オブジェクトの探索 (フロンティア or ウェイポイント) | Action `Explore` | Navigation |
| M4 | **Perception** | RGB-D からのオブジェクト検出・位置推定 | Topic `/detected_objects` | Perception |
| M5 | **Manipulation (Pick)** | SO-101 での把持プランニング & 実行 | Action `Pick` | Manipulation |
| M6 | **Manipulation (Place)** | Drop Zone への配置・ゴミ箱投棄 | Action `Place` | Manipulation |
| M7 | **Task Orchestrator** | 全体ステートマシン、フォールバック管理 | Action `RunTrial` | Integration |
| M8 | **HRI (音声)** | フォールバック指示等の TTS 発話 (加点用) | Topic `/speech_out` | Integration |
| M9 | **Dev Infra** | CI, Docker, rosbag 記録、可視化用 rviz config | — | Infra |

### 3.2 マイルストーン例

| step | 目標 | 完了判定 |
|---|---|---|
| 1 | 環境構築完了・モジュール分割合意 | 全員が `make build` 成功 |
| 2 | M1/M2 スタンドアロン動作 | Rviz 上で目標点までナビ可能 |
| 3 | M4 スタンドアロン動作 | 既知オブジェクトを 80% 以上検出 |
| 4 | M5/M6 スタンドアロン動作 | 手動で置いた物体を pick → place |
| 5 | **M1+M2+M4+M5+M6 統合** (Basic Task ドライラン) | 1 オブジェクトを end-to-end で完遂 |
| 6 | M3 (探索) 統合、M8 (音声) 統合 | Bonus Task をドライランで完遂 |
| 7 | 大会前日: リハーサル 2 回 + フリーズ | ルールブック採点で 60% 以上 |

### 3.3 依存関係グラフ (概略)

```
     ┌────────────┐
     │ M9 Infra   │  (常時)
     └────┬───────┘
          ▼
 ┌────────┴────────┐        ┌────────────┐
 │ M1 Mapping/Loc  │◀──────▶│ M4 Percept │
 └────────┬────────┘        └─────┬──────┘
          ▼                       ▼
 ┌────────────────┐        ┌────────────┐
 │ M2 Navigation  │        │ M5 Pick    │
 └────────┬───────┘        └─────┬──────┘
          │                      ▼
          │               ┌────────────┐
          │               │ M6 Place   │
          │               └─────┬──────┘
          ▼                     ▼
     ┌────────────────────────────┐
     │ M7 Task Orchestrator       │────▶ M8 HRI
     └────────────────────────────┘
                (+ M3 Exploration: Bonus Task 用に M2 と並列)
```

---

## 4. 担当割り振り例 (7 名想定)

**方針**

- 大会得点への寄与が大きい **pick and place** に人員を厚く配置。
- 1 名を **Integration Lead** として全体統合の役割を担うリーダとする。

### 4.1 割り振り表

| # | 役割 | 主担当内容 | 人数目安 |
|---:|---|---|---|
| 1 | **Integration Lead / PM** | M7 (Orchestrator), M9 (CI/rviz) | 1 |
| 2 | Manipulation A | M5 (Pick 動作生成) | 2~3 | 
| 3 | Manipulation B | M6 (Place 動作生成) | 2~3 |
| 3 | Navigation B | M2, M3（基本navigationと、未知物体探索アルゴリズム）| 2~3 |

---

## 5. 統合とテスト

### 5.1 段階的統合

1. **単体テスト** — 各モジュール単体でユニット/機能テスト。
2. **サブシステム統合** — Perception → Manipulation、Nav2 → Orchestrator。
3. **フル統合ドライラン** — ルールブック §6 のシナリオを 1 回通す。
4. **リハーサル** — 15 分制限時間内でのタイムアタック。

---

**関連ドキュメント**

- [competition_rulebook.md](competition_rulebook.md) — 大会公式ルール
- [examples.md](examples.md) — サンプルノード
- [internals.md](internals.md) — 内部設計メモ
