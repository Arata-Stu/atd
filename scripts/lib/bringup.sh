#!/usr/bin/env bash
# 実装済みの構成だけを登録する。各 package の launch 内容はここに複製しない。
# profile / host / package / launch file / 表示名 / 設定TUI種別（タブ区切り）。
atd_bringup_profiles() {
    printf '%s\n' \
        $'input\tjetson\tatd_web_bridge\tinput.launch.py\tJoy・Pico入力、記録待機、観測・設定bridge\tplain' \
        $'bridge\tjetson\tatd_web_bridge\tbridge.launch.py\t観測・設定bridgeのみ（入力は別途起動）\tplain' \
        $'d455\tjetson\tatd_bringup\td455.launch.py\tD455・車両TF（bridgeなし）\tcamera' \
        $'manual\tjetson\tatd_bringup\tmanual.launch.py\tD455・車両TF・CH3切替のDualSense走行・地図用記録待機\tcamera' \
        $'localization\tjetson\tatd_bringup\tlocalization.launch.py\t手動走行・自己位置と前方軌道・操舵と速度制御の監視\tmap' \
        $'esc-trial\tjetson\tatd_bringup\tesc_trial_session\tESC 試験・手順受付と Jetson 端末での実行\ttrial' \
        $'joy-drive\tjetson\tatd_bringup\tjoy_drive.launch.py\tCH3切替のDualSense走行・入力監視と記録待機（カメラなし）\tplain' \
        $'autonomous\tjetson\tatd_bringup\tautonomous.launch.py\tCH3で切り替える低速・前進限定の自動走行（地図・ライン選択）\tmap' \
        $'gazebo\tpc\tatd_bringup\tgazebo.launch.py\tGazebo のステレオ画像と /clock で Visual SLAM を起動\tsimulation'
}

atd_bringup_lookup() {
    local task_requested="$1" task_name task_host task_package task_launch task_label task_setup
    while IFS=$'\t' read -r task_name task_host task_package task_launch task_label task_setup; do
        if [[ "$task_name" == "$task_requested" ]]; then
            printf '%s\t%s\t%s\t%s\n' "$task_host" "$task_package" "$task_launch" "$task_setup"
            return 0
        fi
    done < <(atd_bringup_profiles)
    return 1
}

atd_bringup_list() {
    local task_name task_host task_package task_launch task_label task_setup
    while IFS=$'\t' read -r task_name task_host task_package task_launch task_label task_setup; do
        printf '%-8s %-6s %s\n' "$task_name" "$task_host" "$task_label"
    done < <(atd_bringup_profiles)
}

atd_bringup_choose() {
    local task_current_host="$1" task_choices='' task_choice task_number task_index=0
    local task_name task_host task_package task_launch task_label task_setup
    while IFS=$'\t' read -r task_name task_host task_package task_launch task_label task_setup; do
        [[ "$task_host" == "$task_current_host" ]] || continue
        task_choices+="${task_name}"$'\t'"${task_label} (${task_host})"$'\n'
    done < <(atd_bringup_profiles)
    task_choices+=$'quit\t終了'
    if command -v fzf >/dev/null 2>&1; then
        task_choice="$(printf '%s\n' "$task_choices" | fzf --delimiter=$'\t' --with-nth=2 \
            --height=60% --border --prompt='ATD bringup > ' --header='起動する構成を選択')" || return 1
        printf '%s\n' "${task_choice%%$'\t'*}"
        return 0
    fi
    while IFS=$'\t' read -r task_name task_label; do
        task_index=$((task_index + 1))
        printf '%2d) %s\n' "$task_index" "$task_label" >&2
    done <<< "$task_choices"
    read -r -p '番号（空欄で終了）: ' task_number || return 1
    [[ "$task_number" =~ ^[0-9]+$ && ${#task_number} -le 2 ]] || return 1
    task_number=$((10#$task_number))
    ((task_number >= 1 && task_number <= task_index)) || return 1
    printf '%s\n' "$task_choices" | awk -F '\t' -v item="$task_number" 'NR==item {print $1}'
}
