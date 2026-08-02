# Test Basis: System Diagnostics UI Enhancements (PR-1521)

**Jira Story:** PR-1521  
**Repositories:** `cerebra` (Angular UI) & `pib-backend` (Flask API)  

## Requirements & Acceptance Criteria

### 1. CPU Usage in RAM / Diagnostics Section
- In the System Diagnostics summary table (`#table-system-summary`), the RAM section / memory cell must also display the current CPU usage percentage (e.g. `CPU: xx%` or `CPU Usage: xx%`).
- The backend (`diagnostics_service.py` / `/api/system/diagnostics`) supplies `cpu_percent` / `cpuUsagePercent`.

### 2. Table Header Renaming & Disk Formatting
- Rename header `RAM Memory` -> `Free RAM`.
- Rename header `Disk Space` -> `Free disk space`.
- Reformat the Disk Space display to match the RAM format:  
  `<free disk space> / <total disk space> (xx%)`  
  (e.g., `18.2 GB / 29.1 GB (62%)`).

### 3. Removal of ID Columns in Bricklet Tables
- Remove the first column (`ID` / `brickletNumber`) from the **Servo Bricklets** table (`#table-servo-bricklets`).
- Remove the first column (`ID` / `brickletNumber`) from the **RGB LED Buttons** table (`#table-button-bricklets`).
- Update table headers and `colspan` attributes for empty rows accordingly (from 5 to 4 columns).
