# Teleopit local fixes — export (2026-08-18)

The **active** teleop stack lives in `~/Teleopit` on the laptop — a clone of
[BotRunner64/Teleopit](https://github.com/BotRunner64/Teleopit) with 11 local
commits on branch `local-fixes-2026-08-17`. That clone has no push access to
any remote, so the commits are exported here in two equivalent forms.

- Upstream base: `f9263865c581802ad531854b8e547e2403a945f3` (origin/master)
- Branch tip: `3bb24c228e326afcd6cf2f7cc231c39be8da8875` (2026-08-17)

## To reconstruct the branch

Either apply the patch series:

```bash
git clone https://github.com/BotRunner64/Teleopit.git
cd Teleopit
git checkout -b local-fixes-2026-08-17 f9263865c581802ad531854b8e547e2403a945f3
git am /path/to/teleopit-export/patches/*.patch
```

or fetch from the bundle (same commits, git-native):

```bash
cd Teleopit
git fetch /path/to/teleopit-export/local-fixes-2026-08-17.bundle \
    local-fixes-2026-08-17:local-fixes-2026-08-17
```

Note: patch `0002` (78 MB) and the bundle both carry `PicoBridge_v0.2.1.apk`
(the headset app, com.picobridge.app) which was committed alongside the code —
that is why the files are large.

## What the commits contain (oldest first)

1. **0001** — 2026-08-17 audit fixes: stale-resume reset (RC3), anchor
   deadbands 0.08/0.12 (RC6), alpha rebalance 0.6/0.5 (RC2), pilot height
   1.74, **waist unlock restore** (`action_scale[14]` 0.0 → 0.4386 after the
   waist lock was removed — see `Docs/PROJECT_STATE.md`).
2. **0002** — `root_xy_gain` 1.143 (RC4): amplify pilot root-XY displacement,
   session-anchored, in both real (mp worker) and sim live paths. Includes the
   PicoBridge APK.
3. **0003** — soft-knee deadbands 0.05/0.08 + `root_xy_gain` 1.25.
4. **0004** — JOYSTICK walking mode: right-stick-click toggle from STANDING,
   velocity-command reference synthesis at 50 Hz with caps and deadzone.
5. **0005–0006** — joystick↔mocap round-trip toggling via standing transition.
6. **0007** — kick-direction fix: knee position weight 0/10 → 50 in the
   pico→G1 IK (forward kicks were reaching foot targets via hip abduction);
   joystick caps raised (vx 0.9, vy 0.55, wz 1.1).
7. **0008** — mode-swap collapse fix: reset alignment/velocity on joystick
   exit (stale-yaw lunge), 1.5 s STANDING dwell gate both directions, knee IK
   pos_w 50 → 35.
8. **0009** — strafe on right-stick Y (left-stick X is dead in the app data);
   clearer tracker-asleep gate message.
9. **0010** — foot lift gain 1.6; joystick→teleop swap lands in STANDING and
   requires explicit Y (auto-entry caused falls).
10. **0011** — mocap entry joint blend-in (1.0 s, `mocap_entry_blend_s`) —
    entry was a hard pose snap; strafe moved to grip analogs.

## Robot-side state that is NOT in git

- The Orin (`unitree@192.168.123.164`) runs its own deployed copy of these
  fixes. Rollback archive on the Orin: `~/teleopit_rollback_2026-08-17.tgz`.
- `/home/unitree/discovery_relay.py` on the Orin (crontab `@reboot`) —
  broadcasts `192.168.123.2|63901` into `10.42.0.255` so the headset on the
  `g1-teleop` AP can find the laptop bridge (the stock UDP discovery cannot
  cross the Orin NAT).
