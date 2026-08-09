# Mobile Lab Disturbance and Local HTTPS Design

## Context

The first phone rehearsal exposed two concrete gaps:

- the LAN URL was plain HTTP, so mobile browsers did not expose the motion
  sensor path;
- the existing deterministic sinusoidal disturbance produced an unattended
  20-second score of 959/1000 with only about 1.1° maximum tilt.

The approved difficulty target is an unattended average score of 400–550.
The application remains an educational browser simulation and must not gain
any path to a real drone, firmware, motor, serial device, USB device,
Bluetooth device, or UDP control surface.

## Disturbance model

### Selected approach

Use a seeded, deterministic wind model composed of:

1. slowly varying cross-axis wind;
2. smoothed pseudo-random turbulence;
3. intermittent gust envelopes with independent Roll and Pitch direction.

Do not use `Math.random()` inside the physics step. A 32-bit challenge seed
initializes a small local pseudo-random generator, and its evolving state is
stored in the challenge state. Identical seed, input sequence, and fixed time
steps must produce byte-for-byte identical terminal state. Different seeds
must produce observably different attitude histories.

The browser chooses a fresh seed once per challenge or retry using
`crypto.getRandomValues`. Randomness therefore changes the experience between
attempts without making the simulation untestable. The server never computes
physics and the score calculation module remains unchanged.

### Difficulty and fairness contracts

A fixed calibration bank of at least eight seeds defines the behavioral
contract:

- no-input 20-second mean score: 400–550;
- every calibration seed remains within a bounded playable range rather than
  containing a single unfair outlier;
- visible attitude excursion is materially larger than the old 1.1° result;
- a literal, independently specified proportional-derivative test pilot can
  recover substantially better scores than no input, proving that the game is
  demanding rather than arbitrary;
- scores remain within 0–1000 and the challenge still ends at exactly 20
  seconds.

These are behavior tests, not assertions on private force constants. Force,
damping, gust probability, and envelope constants may be tuned only to meet
the contracts above.

### Interfaces

- `createChallengeState({ seed } = {})` creates a complete deterministic state.
- `stepChallenge(state, input, dt)` advances that state without mutation.
- `restartChallenge(previousState, { seed } = {})` resets all physics and score
  fields while accepting the next attempt's seed.
- `app.mjs` supplies a fresh unsigned 32-bit seed for every first attempt and
  retry.

No new UI control is required. The existing artificial horizon, Roll·Pitch,
stability, score, and remaining time already make the stronger disturbance
visible.

## Local trusted HTTPS

Use the user-controlled hostname `uos-drone.kro.kr` and a public-trust
Let's Encrypt certificate issued with DNS-01 validation:

1. publish an A record resolving `uos-drone.kro.kr` to the current rehearsal
   LAN address `192.168.0.6`;
2. create the temporary `_acme-challenge.uos-drone.kro.kr` TXT record supplied
   by the ACME client;
3. verify the TXT answer from public DNS before continuing issuance;
4. store the certificate and private key outside the repository in a
   user-readable, permission-restricted Let's Encrypt configuration directory;
5. launch the existing standard-library server on port 8443 with `--cert` and
   `--key`;
6. use `https://uos-drone.kro.kr:8443/` for students and
   `https://uos-drone.kro.kr:8443/presenter.html` for the presenter.

DNS-01 avoids public router port forwarding. The A record is rehearsal-network
specific and must be updated when the LAN address changes. No certificate,
private key, ACME account material, or DNS credential may be written to or
committed in the repository.

## Verification boundary

Automated tests prove deterministic seeded physics, difficulty envelopes,
controllability, restart behavior, and existing browser/server regressions.
The live HTTPS check proves certificate presentation, hostname matching, and
page/API availability from this host. The user must still verify the actual
phone's certificate trust, DeviceOrientation permission prompt, physical axes,
sample rate, and event Wi-Fi behavior. None of this is real-flight proof.
