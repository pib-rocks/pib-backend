# Implementation Plan: PR-1521 System Diagnostics UI Enhancements

**Jira Story:** PR-1521  
**Repositories:** `cerebra` & `pib-backend`  

## Objectives & Scope

### Backend (`pib-backend`)
- In `pib_api/flask/service/diagnostics_service.py`: ensure `cpu_percent` or `cpuUsagePercent` is included in the `/api/system/diagnostics` summary payload.

### Frontend (`cerebra`)
- In `src/app/system/diagnostics/diagnostics.component.html`:
  1. Rename header `RAM Memory` -> `Free RAM`.
  2. Rename header `Disk Space` -> `Free disk space`.
  3. Format disk space as `<free> / <total> (xx%)`.
  4. Display CPU usage percentage inside the Free RAM / memory cell.
  5. Remove the `ID` column (`<th>ID</th>` and `<td>{{ b.brickletNumber }}</td>`) from `#table-servo-bricklets`.
  6. Remove the `ID` column (`<th>ID</th>` and `<td>{{ b.brickletNumber }}</td>`) from `#table-button-bricklets`.
  7. Update `colspan` from `5` to `4` for empty data rows in both tables.

### Testing
- Update Angular component unit tests in `diagnostics.component.spec.ts`.
- Update / add Playwright E2E UI tests.
