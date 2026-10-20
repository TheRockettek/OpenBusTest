# Audio configuration: incomplete support

Scope: player/AI sound references in `.bus` files, sound CFG records, playback/trigger behavior, and audio file decoding. OpenBus has a working player sound path, but it does not implement the complete OMSI sound contract.

## Sound CFG records

| Keyword / family | Status | Current boundary |
| --- | --- | --- |
| `[sound]`, `[trigger]` | Active subset | Files, base gain, trigger aliases, and one-shot/loop playback are supported. An unknown trigger is logged and does not play. This does not mean every trigger name used by OMSI scripts exists in the loaded bank. |
| `[loop]`, `[looped]`, `[noloop]` | Active subset | These markers change the configured sound's loop state. Full native trigger/channel ownership parity is not established. |
| `[viewpoint]` | Active subset | Sound playback uses the exterior/interior/non-player mask. Exterior sounds can also reach the player interior at the script's `Snd_OutsideVol` gain; zero keeps them excluded. |
| `[3d]` | Partial | Source position and maximum distance are loaded into the OpenAL source. Full OMSI spatialization, attenuation, orientation, and per-record field semantics are not verified. |
| `[volcurve]`, `[pnt]` | Partial | Curves affect triggered sounds and untriggered ambient loops. Ambient curves use script variables; complete curve/control semantics and OMSI parity remain unverified. |
| `[conditionSingle]` | Active subset | A single variable comparison can gate a sound; comparison codes 0–5 are supported. More complex condition/group logic is not provided. |
| `[loopsound]` | Partial | Untriggered ambient loops can respond to a control variable, curve, and condition. Only a subset of the serialized fields is used; sample-rate/mode and other native loop semantics are not fully implemented. |
| Start/loop/end filename families | Heuristic | `_start`, `_loop`/`-loop`, and `_end` naming is used to infer loop pairing/stopping. This is an OpenBus heuristic, not confirmed OMSI behavior. |
| Same-file retrigger | Partial | OpenBus defaults to a `KeepPlaying` policy and updates a live source; exact native OMSI retrigger behavior has not been validated. |

Unknown authored triggers remain a content gap rather than a missing CFG keyword. For example, the supplied London run logs `ev_bremse_hup` as an unknown trigger, while a configured `ev_schluessel_steck` playback request succeeds in that run.

The DL05's sound-volume script derives `Snd_OutsideVol` from its doors and driver
window. Ambient exterior loops update their gain as those openings move and stop
when it returns to zero; newly triggered exterior sounds use the same multiplier.
Interior sounds and the exterior camera retain their configured gain. This is a
volume mix, not acoustic occlusion or low-pass filtering.

## BUS audio references

- `[sound]` loads the player sound CFG and is connected to the player sound bank.
- `[sound_ai]` is parsed and its path retained, but AI vehicles do not select/use an AI sound bank.
- Sound selection for AI vehicles and the complete OMSI AI sound trigger/state producer set are not implemented.

## Audio file decoding and playback limits

The current backend decodes RIFF/WAVE PCM (8-, 16-, 24-, or 32-bit) and 32-bit IEEE-float audio, with mono or stereo channels, then plays through OpenAL. Compressed/non-WAV formats such as Ogg Vorbis or MP3 are not decoded by this path. OpenAL listener/source positions and optional Doppler are active; these backend capabilities do not establish complete OMSI sound-CFG semantics.

## Evidence and related pages

Implementation references: [VehicleSoundBank.cpp](../src/audio/VehicleSoundBank.cpp), [VehicleSoundBank.h](../src/audio/VehicleSoundBank.h), [SoundEngine.cpp](../src/audio/SoundEngine.cpp), and [VehicleConfigLoader.cpp](../src/config/VehicleConfigLoader.cpp). Regression coverage is in [SoundEngineProbe.cpp](../tests/SoundEngineProbe.cpp). BUS/model limitations are documented in [BUS status](IMPLEMENTATION_STATUS_BUS.md); see the [configuration status index](CONFIGURATION_IMPLEMENTATION_STATUS.md) for all file families.
