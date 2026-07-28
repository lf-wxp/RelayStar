# Multi-stage build for the RelayStar MQTT broker.
# Build context is the repository root:  docker build -f docker/Dockerfile .

# Chainguard's Wolfi-based Rust image is maintained to carry zero known CVEs
# (glibc, so the resulting binary runs on the distroless cc runtime below).
FROM cgr.dev/chainguard/rust:latest AS builder
USER root
WORKDIR /build

# Copy the whole workspace so path dependencies (relaystar-proto) resolve.
# (rust-toolchain.toml is intentionally omitted: the image's stable toolchain
# builds the host broker fine, and it avoids a rustup dependency.)
COPY Cargo.toml Cargo.lock* ./
COPY crates/relaystar-proto ./crates/relaystar-proto
COPY crates/broker ./crates/broker

# Firmware crates are Xtensa-only and excluded from the workspace, so they are
# intentionally not copied into the broker image.
RUN cargo build --release -p relaystar-broker

# Distroless: no shell, no package manager, minimal userland -> tiny CVE surface.
# The `cc` variant ships glibc + libssl for the dynamically-linked Rust binary,
# and `:nonroot` runs as an unprivileged user (uid 65532).
FROM gcr.io/distroless/cc-debian12:nonroot AS runtime
WORKDIR /app

COPY --from=builder /build/target/release/relaystar-broker /usr/local/bin/relaystar-broker
COPY docker/rumqttd.toml /app/rumqttd.toml

ENV RELAYSTAR_BROKER_CONFIG=/app/rumqttd.toml
ENV RUST_LOG=info

EXPOSE 1883 1884 8080 3030
USER nonroot

ENTRYPOINT ["/usr/local/bin/relaystar-broker"]
