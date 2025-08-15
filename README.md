# Team 581's Offseason 2025 Robot Code Monorepo

[![CI](https://github.com/team581/offseason-2025/actions/workflows/ci.yml/badge.svg)](https://github.com/team581/offseason-2025/actions/workflows/ci.yml)

[Team 581](https://github.com/team581)'s 2025 offseason robot code monorepo.

## Project Structure

This repository is organized as a Gradle monorepo with the following projects:

- **`shared/`** - Shared utility library
- **`comp-bot/`** - Competition robot code
- **`offseason-bot/`** - Offseason bot robot code
- **`new-member-bot/`** - New member bot robot code

## Building and running

### Build

```sh
# Build all projects
./gradlew build

# Build specific project
./gradlew comp-bot:build
```

### Deploy to roboRIO

```sh
./gradlew comp-bot:deploy
```

### Running the simulator

```sh
# Run simulator for specific project
./gradlew comp-bot:simulateJava
```

### Running tests

```sh
# Run all tests
./gradlew test

# Run tests for specific project
./gradlew comp-bot:test
```

### Code formatting

```sh
# Check formatting
./gradlew spotlessCheck

# Apply formatting
./gradlew spotlessApply
```
