# Stamped per-cycle correction, used as the stream-mode command topic of
# CartesianCorrectionController (spec section 7.7). The controller zeroes
# its output when the stamp is older than the configured command timeout.
Header header
CartesianCorrection correction
